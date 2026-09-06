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

// Velocity iterations so simultaneous contacts converge together.
// A single contact converges on the first pass.
constexpr int kVelocityIterations = 8;
// Approach speeds below this bounce as if restitution were 0, killing
// micro-bounce near rest.
constexpr float kRestitutionThreshold = 0.5f;
// Baumgarte correction: fix a fraction of penetration per step, with a small
// allowed overlap.
constexpr float kSlop = 0.005f;
constexpr float kCorrectionPercent = 0.4f;

// Union-find over body indices, grouping contact-connected bodies into sleep
// islands.
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
        // A sleeping body is not integrated, so it cannot sink while asleep.
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
    // Direct member writes, not the public setters: those wake the body and
    // reset its sleep timer, so resting bodies would never sleep.
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
    if (n.Length() < 0.5f) return false;  // degenerate normal, skip

    c.ia = ia;
    c.ib = ib;
    c.n = n;
    c.rA = d.m_contactPoint - a.GetPosition();
    c.rB = d.m_contactPoint - b.GetPosition();

    // Orthonormal tangent basis derived from n, so warm-started tangents
    // stay consistent next step.
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

    // Penetration for positional correction, recovered from the bodies'
    // geometry along n since distance is not a uniform depth across pairs.
    // Plane pair: penetration = support - distance.
    if (a.IsPlane() || b.IsPlane()) {
        const PhysicsObject& finite = a.IsPlane() ? b : a;
        c.penetration = finite.SupportRadius(n) - d.distance;
    } else {
        float centerGap{std::abs((b.GetPosition() - a.GetPosition()).Dot(n))};
        c.penetration = a.SupportRadius(n) + b.SupportRadius(n) - centerGap;
    }

    if (speculative) {
        // CCD: separated but maybe closing. Cap approach to the current gap in
        // one step, target vn = -gap/dt, no restitution. If the gap reads gone
        // the target is 0.
        float gap = std::max(-c.penetration, 0.0f);
        c.targetVn = -gap / m_lastDelta;
    } else {
        // Restitution target from the pre-solve approach speed, captured before
        // any impulses so it is not double counted. Only approaching contacts
        // bounce.
        float vn = (ContactVelocity(b, c.rB) - ContactVelocity(a, c.rA)).Dot(n);
        c.targetVn = (vn < -kRestitutionThreshold) ? -m_restitution * vn : 0.0f;
    }
    return true;
}

void PhysicsEngine::WarmStart(Contact& c) {
    // Speculative CCD contacts skip warm starting. They skip friction too, so a
    // reused tangent impulse would never be corrected and would pump energy.
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

    // Normal impulse toward the restitution target. The accumulated Pn is
    // clamped to Pn >= 0, so iterations may correct downward and still converge.
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

    // Friction kills tangential slip, tangent impulse clamped to the cone
    // |Pt| <= mu*Pn. Speculative contacts get none, surfaces not touching yet.
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
    // Don't push a speculative, not-yet-touching pair apart; its gap recovery
    // can misreport overlap.
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

    // Islands are connected components over dynamic contact edges. Statics join
    // no island, so a pile resting on the floor can still sleep.
    UnionFind uf(n);
    for (const auto& c : contacts)
        if (!m_objects[c.ia].IsStatic() && !m_objects[c.ib].IsStatic())
            uf.unite(static_cast<int>(c.ia), static_cast<int>(c.ib));

    // Any island with an awake member wakes wholly: contact-wake plus neighbour
    // propagation.
    std::vector<bool> rootAwake(n, false);
    for (std::size_t i = 0; i < n; ++i)
        if (!m_objects[i].IsStatic() && m_objects[i].m_awake)
            rootAwake[uf.find(static_cast<int>(i))] = true;
    for (std::size_t i = 0; i < n; ++i)
        if (!m_objects[i].IsStatic() && !m_objects[i].m_awake &&
            rootAwake[uf.find(static_cast<int>(i))])
            m_objects[i].WakeUp();

    // Contacts with no awake body are dropped from the solve.
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

    // Rest timers accumulate while the body is slow in both linear and angular
    // speed, reset when it moves.
    for (auto& o : m_objects) {
        if (o.IsStatic() || !o.m_awake) continue;
        bool resting = o.m_velocity.Length() <= m_sleepLinearThreshold &&
                       o.m_angularVelocity.Length() <= m_sleepAngularThreshold;
        o.m_sleepTime = resting ? o.m_sleepTime + m_lastDelta : 0.0f;
    }

    // An island sleeps only when every member has rested long enough. Sleeping
    // as one avoids the flip-flop where a settling neighbour re-wakes an early
    // sleeper.
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

    // Broad phase: bound each body with an AABB and cull to candidate overlap
    // pairs. A CCD body's bound is swept forward so an about-to-touch pair is
    // still reported.
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

    // Narrow phase: gather every contact before solving, so the solver sees all
    // of a stacked body's contacts together instead of one pair at a time.
    std::vector<Contact> contacts;
    contacts.reserve(pairs.size());
    for (const auto& [i, j] : pairs) {
        PhysicsObject& a = m_objects[i];
        PhysicsObject& b = m_objects[j];
        if (a.IsStatic() && b.IsStatic()) continue;  // nothing can move

        WorldShape sa = a.GetWorldShape();
        WorldShape sb = b.GetWorldShape();
        // std::visit picks the collision specialization for the shape pair;
        // m_normal points a to b.
        IntersectData d = std::visit(
            [](const auto& x, const auto& y) { return collision(x, y); }, sa,
            sb);

        // Real contact on a hit, or a speculative contact when a CCD body's
        // swept pair has not touched yet.
        bool speculative = !d.m_doesIntersect && (ccd[i] || ccd[j]);
        Contact c;
        if ((d.m_doesIntersect || speculative) &&
            BuildContact(static_cast<std::size_t>(i),
                         static_cast<std::size_t>(j), d, speculative, c))
            contacts.push_back(c);
    }

    // Wake pass: islands touched by an awake body wake, and fully-sleeping
    // contacts drop out.
    WakeIslands(contacts);

    // Warm start from last step's impulses, then iterate the velocity solver.
    for (auto& c : contacts) WarmStart(c);
    for (int iter = 0; iter < kVelocityIterations; ++iter)
        for (auto& c : contacts) SolveVelocity(c);

    // Positional correction after the velocity solve.
    for (const auto& c : contacts) CorrectPositions(c);

    // Sleep pass: rest timers advance on post-solve velocities; an island
    // sleeps when all members qualify.
    SleepIslands(contacts);

    // Cache this step's impulses for next-step warm starting. Speculative
    // contacts are excluded; their Pn is a CCD brake, not a contact impulse.
    m_impulseCache.clear();
    for (const auto& c : contacts)
        if (!c.speculative)
            m_impulseCache[{c.ia, c.ib}] = CachedImpulse{c.Pn, c.Pt1, c.Pt2};
}

}  // namespace Physics
