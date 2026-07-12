#include "physicsEngine.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>
#include <variant>
#include <vector>

#include "broadphase.h"
#include "collisionDispatch.h"
#include "obbCollision.h"  // declares collision<OBB, X> before std::visit uses them

namespace Physics {
namespace {

// Solver tuning. Velocity iterations let simultaneous contacts (stacks) push
// back on each other until the whole set converges; a single contact converges
// on the first iteration, reproducing the old one-shot impulse exactly.
constexpr int kVelocityIterations = 8;
// Approach speeds below this bounce as if restitution were 0 — kills the
// endless micro-bounce of a nearly-resting body without affecting real impacts.
constexpr float kRestitutionThreshold = 0.5f;
// Positional (Baumgarte) correction: fix a fraction of the penetration per
// step, with a small permitted overlap so resting stacks don't jitter.
constexpr float kSlop = 0.005f;
constexpr float kCorrectionPercent = 0.4f;

// Tiny union-find over body indices, used to group contact-connected dynamic
// bodies into islands that sleep/wake as one (SLEEP-1).
struct UnionFind {
    std::vector<int> parent;
    explicit UnionFind(std::size_t n) : parent(n) {
        std::iota(parent.begin(), parent.end(), 0);
    }
    int find(int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];  // path halving
            x = parent[x];
        }
        return x;
    }
    void unite(int a, int b) { parent[find(a)] = find(b); }
};

}  // namespace

std::size_t PhysicsEngine::AddObject(const PhysicsObject& object) {
    m_objects.push_back(object);
    return m_objects.size() - 1;
}

void PhysicsEngine::Simulate(float delta) {
    if (delta > 0.0f) m_lastDelta = delta;
    for (auto& obj : m_objects) {
        // A sleeping body is not integrated at all — no gravity accumulation,
        // no motion, so it cannot sink while asleep (SLEEP-1).
        if (m_sleepingEnabled && !obj.m_awake) continue;
        obj.Integrate(delta, m_gravity);
    }
}

void PhysicsEngine::SetSleepingEnabled(bool on) {
    m_sleepingEnabled = on;
    // Turning sleeping off must leave no body stranded asleep.
    if (!on)
        for (auto& obj : m_objects) obj.WakeUp();
}

bool PhysicsEngine::CcdActive(const PhysicsObject& o) const {
    if (o.IsStatic()) return false;
    return o.IsContinuous() ||
           o.GetVelocity().Length() * m_lastDelta > m_ccdSpeedThreshold;
}

Vector3f PhysicsEngine::ContactVelocity(const PhysicsObject& o,
                                        const Vector3f& r) {
    return o.GetVelocity() + o.GetAngularVelocity().Cross(r);
}

void PhysicsEngine::ApplyImpulse(PhysicsObject& o, const Vector3f& r,
                                 const Vector3f& impulse, float sign) {
    // Direct member writes (friend), NOT the public setters: SetVelocity/
    // SetAngularVelocity wake the body and reset its sleep timer, which would
    // stop resting bodies from ever falling asleep (the solver nudges them
    // every step). Waking is handled explicitly by the island wake pass.
    Vector3f p{impulse * sign};
    o.m_velocity += p * o.GetInvMass();
    o.m_angularVelocity += o.ApplyInverseInertia(r.Cross(p));
}

bool PhysicsEngine::BuildContact(std::size_t ia, std::size_t ib,
                                 const IntersectData& d, bool speculative,
                                 Contact& c) const {
    const PhysicsObject& a = m_objects[ia];
    const PhysicsObject& b = m_objects[ib];
    const Vector3f& n = d.m_normal;       // unit, points a -> b on a hit
    if (n.Length() < 0.5f) return false;  // degenerate normal — skip

    c.ia = ia;
    c.ib = ib;
    c.n = n;
    c.rA = d.m_contactPoint - a.GetPosition();
    c.rB = d.m_contactPoint - b.GetPosition();

    // Orthonormal tangent basis, a deterministic function of n alone (so
    // warm-started tangent impulses land in the same directions next step).
    Vector3f ref =
        std::abs(n.GetX()) < 0.9f ? Vector3f(1, 0, 0) : Vector3f(0, 1, 0);
    c.t1 = n.Cross(ref).Normalized();
    c.t2 = n.Cross(c.t1);

    // Effective mass along a direction dir:
    //   k = invMassSum + dir . ( Iinv_a(rA x dir) x rA + Iinv_b(rB x dir) x rB
    //   )
    const float invSum = a.GetInvMass() + b.GetInvMass();
    auto effectiveMass = [&](const Vector3f& dir) {
        Vector3f angA{a.ApplyInverseInertia(c.rA.Cross(dir)).Cross(c.rA)};
        Vector3f angB{b.ApplyInverseInertia(c.rB.Cross(dir)).Cross(c.rB)};
        float k = invSum + dir.Dot(angA + angB);
        return k > 0.0f ? 1.0f / k : 0.0f;
    };
    c.normalMass = effectiveMass(n);
    c.tangentMass1 = effectiveMass(c.t1);
    c.tangentMass2 = effectiveMass(c.t2);

    c.friction = std::sqrt(a.GetFriction() * b.GetFriction());
    c.speculative = speculative;

    // Penetration for positional correction (negative = separation gap). The
    // detector's `distance` is not a uniform penetration depth across shape
    // pairs, so recover it from the bodies' own geometry along n. For a plane
    // pair, `distance` IS the finite body's center-to-plane distance, so
    // penetration = support - distance.
    if (a.IsPlane() || b.IsPlane()) {
        const PhysicsObject& finite = a.IsPlane() ? b : a;
        c.penetration = finite.SupportRadius(n) - d.distance;
    } else {
        float centerGap{std::abs((b.GetPosition() - a.GetPosition()).Dot(n))};
        c.penetration = a.SupportRadius(n) + b.SupportRadius(n) - centerGap;
    }

    if (speculative) {
        // CCD: the pair is separated but (possibly) closing fast. Allow it to
        // approach by at most the current gap in one step: target vn = -gap/dt.
        // No restitution before real contact. If the support-based recovery
        // says the gap is already gone (conservative for corner approaches),
        // the target degrades to 0 — a resting-style constraint, never a push.
        float gap = std::max(-c.penetration, 0.0f);
        c.targetVn = -gap / m_lastDelta;
    } else {
        // Restitution target from the PRE-solve approach speed (captured
        // before warm starting or any other contact's impulses so it is not
        // double counted). Only approaching contacts bounce; slow ones don't.
        float vn = (ContactVelocity(b, c.rB) - ContactVelocity(a, c.rA)).Dot(n);
        c.targetVn = (vn < -kRestitutionThreshold) ? -m_restitution * vn : 0.0f;
    }
    return true;
}

void PhysicsEngine::WarmStart(Contact& c) {
    // Speculative (CCD) contacts are excluded from warm starting entirely.
    // They skip the friction solve, so a warm-started tangent impulse would
    // never be corrected — after a real landing caches large friction
    // impulses, re-applying them on the next step's speculative contact is a
    // free kick at a long lever arm (the sphere-plane contact point sits on
    // the plane, far below an airborne center), which pumped energy into
    // bouncing CCD bodies (|v| 20->90, sign-reversing |w|). A cold-started
    // speculative contact converges in one iteration anyway.
    if (c.speculative) return;
    auto it = m_impulseCache.find({c.ia, c.ib});
    if (it == m_impulseCache.end()) return;
    c.Pn = it->second.Pn;
    c.Pt1 = it->second.Pt1;
    c.Pt2 = it->second.Pt2;
    Vector3f p{c.n * c.Pn + c.t1 * c.Pt1 + c.t2 * c.Pt2};
    ApplyImpulse(m_objects[c.ia], c.rA, p, -1.0f);
    ApplyImpulse(m_objects[c.ib], c.rB, p, +1.0f);
}

void PhysicsEngine::SolveVelocity(Contact& c) {
    PhysicsObject& a = m_objects[c.ia];
    PhysicsObject& b = m_objects[c.ib];

    // Normal: drive the separating speed to the restitution target, with the
    // TOTAL accumulated impulse clamped to be repulsive (Pn >= 0) — individual
    // iterations may correct downward, which is what lets warm starting and
    // stacked contacts converge instead of over-pushing.
    {
        float vn =
            (ContactVelocity(b, c.rB) - ContactVelocity(a, c.rA)).Dot(c.n);
        float dPn = (c.targetVn - vn) * c.normalMass;
        float oldPn = c.Pn;
        c.Pn = std::max(oldPn + dPn, 0.0f);
        dPn = c.Pn - oldPn;
        Vector3f p{c.n * dPn};
        ApplyImpulse(a, c.rA, p, -1.0f);
        ApplyImpulse(b, c.rB, p, +1.0f);
    }

    // Friction: kill tangential slip at the contact, with the accumulated
    // tangent impulse clamped to the Coulomb cone |Pt| <= mu * Pn. Speculative
    // contacts get none — the surfaces are not touching yet.
    if (c.friction > 0.0f && !c.speculative) {
        const float maxPt = c.friction * c.Pn;
        auto solveTangent = [&](const Vector3f& t, float tangentMass,
                                float& Pt) {
            float vt =
                (ContactVelocity(b, c.rB) - ContactVelocity(a, c.rA)).Dot(t);
            float dPt = -vt * tangentMass;
            float oldPt = Pt;
            Pt = std::clamp(oldPt + dPt, -maxPt, maxPt);
            dPt = Pt - oldPt;
            Vector3f p{t * dPt};
            ApplyImpulse(a, c.rA, p, -1.0f);
            ApplyImpulse(b, c.rB, p, +1.0f);
        };
        solveTangent(c.t1, c.tangentMass1, c.Pt1);
        solveTangent(c.t2, c.tangentMass2, c.Pt2);
    }
}

void PhysicsEngine::CorrectPositions(const Contact& c) {
    // Never positionally push a speculative (not-yet-touching) pair apart —
    // the conservative gap recovery can report overlap for corner approaches.
    if (c.speculative) return;
    PhysicsObject& a = m_objects[c.ia];
    PhysicsObject& b = m_objects[c.ib];
    const float invSum = a.GetInvMass() + b.GetInvMass();
    if (invSum <= 0.0f) return;

    float corr{std::max(c.penetration - kSlop, 0.0f) / invSum *
               kCorrectionPercent};
    if (corr > 0.0f) {
        Vector3f push{c.n * corr};
        a.SetPosition(a.GetPosition() - push * a.GetInvMass());
        b.SetPosition(b.GetPosition() + push * b.GetInvMass());
    }
}

void PhysicsEngine::WakeIslands(std::vector<Contact>& contacts) {
    if (!m_sleepingEnabled) return;
    const std::size_t n = m_objects.size();

    // Islands = connected components over dynamic-dynamic contact edges.
    // Statics join no island (and wake nobody): a pile resting on the immovable
    // floor must still be able to sleep.
    UnionFind uf(n);
    for (const auto& c : contacts)
        if (!m_objects[c.ia].IsStatic() && !m_objects[c.ib].IsStatic())
            uf.unite(static_cast<int>(c.ia), static_cast<int>(c.ib));

    // Any island containing an awake member wakes wholly — this is both the
    // "contact with an awake body" trigger (a thrown ball touching a sleeping
    // pile) and the neighbour-propagation across the island.
    std::vector<bool> rootAwake(n, false);
    for (std::size_t i = 0; i < n; ++i)
        if (!m_objects[i].IsStatic() && m_objects[i].m_awake)
            rootAwake[uf.find(static_cast<int>(i))] = true;
    for (std::size_t i = 0; i < n; ++i)
        if (!m_objects[i].IsStatic() && !m_objects[i].m_awake &&
            rootAwake[uf.find(static_cast<int>(i))])
            m_objects[i].WakeUp();

    // Fully-sleeping contacts (sleeping-sleeping or sleeping-static) are not
    // solved — that is the entire point of sleeping.
    contacts.erase(std::remove_if(contacts.begin(), contacts.end(),
                                  [&](const Contact& c) {
                                      return !m_objects[c.ia].m_awake &&
                                             !m_objects[c.ib].m_awake;
                                  }),
                   contacts.end());
}

void PhysicsEngine::SleepIslands(const std::vector<Contact>& contacts) {
    if (!m_sleepingEnabled) return;
    const std::size_t n = m_objects.size();

    // Rest timers: accumulate while the body is slow (both linear AND angular
    // below threshold), reset the moment it moves.
    for (auto& o : m_objects) {
        if (o.IsStatic() || !o.m_awake) continue;
        bool resting = o.m_velocity.Length() <= m_sleepLinearThreshold &&
                       o.m_angularVelocity.Length() <= m_sleepAngularThreshold;
        o.m_sleepTime = resting ? o.m_sleepTime + m_lastDelta : 0.0f;
    }

    // An island sleeps only when EVERY member has rested long enough (islands
    // sleep as one — otherwise one body dozing off early would be re-woken by
    // its still-settling neighbour and the island would flip-flop forever).
    // After the wake pass islands are homogeneous, so only awake ones matter.
    UnionFind uf(n);
    for (const auto& c : contacts)
        if (!m_objects[c.ia].IsStatic() && !m_objects[c.ib].IsStatic())
            uf.unite(static_cast<int>(c.ia), static_cast<int>(c.ib));

    std::vector<bool> rootBlocked(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        const PhysicsObject& o = m_objects[i];
        if (!o.IsStatic() && o.m_awake && o.m_sleepTime < m_timeToSleep)
            rootBlocked[uf.find(static_cast<int>(i))] = true;
    }
    for (std::size_t i = 0; i < n; ++i) {
        PhysicsObject& o = m_objects[i];
        if (o.IsStatic() || !o.m_awake) continue;
        if (rootBlocked[uf.find(static_cast<int>(i))]) continue;
        o.m_awake = false;  // sleep: freeze all motion
        o.m_velocity = Vector3f(0, 0, 0);
        o.m_angularVelocity = Vector3f(0, 0, 0);
    }
}

void PhysicsEngine::HandleCollisions() {
    const std::size_t n = m_objects.size();
    if (n < 2) {
        m_impulseCache.clear();
        return;
    }

    // Broad phase: bound every body with an AABB, then cull to candidate
    // overlap pairs so the exact narrow phase only runs where it can matter.
    // A CCD-active body's bound is SWEPT forward by one step's displacement so
    // an about-to-happen pair is still reported even though the shapes don't
    // overlap yet (speculative contacts, CCD-1).
    std::vector<AABB> bounds;
    std::vector<bool> ccd(n);
    bounds.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        AABB box = m_objects[i].GetWorldAABB();
        ccd[i] = CcdActive(m_objects[i]);
        if (ccd[i]) {
            Vector3f disp{m_objects[i].GetVelocity() * m_lastDelta};
            Vector3f mn{box.getMin()}, mx{box.getMax()};
            box = AABB(Vector3f(mn.GetX() + std::min(disp.GetX(), 0.0f),
                                mn.GetY() + std::min(disp.GetY(), 0.0f),
                                mn.GetZ() + std::min(disp.GetZ(), 0.0f)),
                       Vector3f(mx.GetX() + std::max(disp.GetX(), 0.0f),
                                mx.GetY() + std::max(disp.GetY(), 0.0f),
                                mx.GetZ() + std::max(disp.GetZ(), 0.0f)));
        }
        bounds.push_back(box);
    }

    const std::vector<std::pair<int, int>> pairs =
        Broadphase::sweepAndPrune(bounds);

    // Narrow phase: gather every real contact BEFORE solving, so the solver
    // sees all of them together (a body in two contacts — e.g. mid-stack —
    // must satisfy both, which one-pair-at-a-time resolution cannot do).
    std::vector<Contact> contacts;
    contacts.reserve(pairs.size());
    for (const auto& [i, j] : pairs) {
        PhysicsObject& a = m_objects[i];
        PhysicsObject& b = m_objects[j];
        if (a.IsStatic() && b.IsStatic()) continue;  // nothing can move

        WorldShape sa = a.GetWorldShape();
        WorldShape sb = b.GetWorldShape();
        // std::visit picks the collision<A,B>() specialization for the concrete
        // shape pair; m_normal points a -> b (see collisionDispatch.cpp).
        IntersectData d = std::visit(
            [](const auto& x, const auto& y) { return collision(x, y); }, sa,
            sb);

        // Real contact on a hit; SPECULATIVE contact when a CCD-active body is
        // in a candidate pair that has not touched yet (the swept bound put it
        // here) — the solver then caps its approach speed at gap/dt.
        bool speculative = !d.m_doesIntersect && (ccd[i] || ccd[j]);
        Contact c;
        if ((d.m_doesIntersect || speculative) &&
            BuildContact(static_cast<std::size_t>(i),
                         static_cast<std::size_t>(j), d, speculative, c))
            contacts.push_back(c);
    }

    // SLEEP-1 wake pass (no-op when sleeping is disabled): islands touched by
    // an awake body wake wholly, and contacts between still-sleeping bodies
    // (or sleeping-vs-static) drop out of the solve below.
    WakeIslands(contacts);

    // Warm start from last step's impulses, then iterate the velocity solver.
    for (auto& c : contacts) WarmStart(c);
    for (int iter = 0; iter < kVelocityIterations; ++iter)
        for (auto& c : contacts) SolveVelocity(c);

    // Positional correction after the velocity solve.
    for (const auto& c : contacts) CorrectPositions(c);

    // SLEEP-1 sleep pass (no-op when disabled): rest timers advance on the
    // post-solve velocities; an island sleeps once all its members qualify.
    SleepIslands(contacts);

    // Remember this step's accumulated impulses for next-step warm starting.
    // Speculative contacts don't contribute: their Pn is a CCD braking
    // constraint, not a contact impulse — inheriting it as a warm start on the
    // following real contact would misrepresent the landing.
    m_impulseCache.clear();
    for (const auto& c : contacts)
        if (!c.speculative)
            m_impulseCache[{c.ia, c.ib}] = CachedImpulse{c.Pn, c.Pt1, c.Pt2};
}

}  // namespace Physics
