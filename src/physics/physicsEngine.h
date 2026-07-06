#pragma once
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "../core/math3d.h"
#include "intersectData.h"
#include "physicsObject.h"

// The physics simulation loop.
//
// PhysicsEngine owns a flat list of PhysicsObjects. Each fixed step:
//   Simulate(dt)        -> integrate every body (semi-implicit Euler + gravity)
//   HandleCollisions()  -> broad-phase cull, narrow-phase collision<>(), then a
//                          SEQUENTIAL-IMPULSE contact solver: all contacts are
//                          gathered first, warm-started from the previous step's
//                          accumulated impulses, then iterated a few times so
//                          simultaneous contacts (stacks!) converge together.
//                          Each contact solves a normal impulse (accumulated,
//                          clamped >= 0, with restitution) and a Coulomb
//                          friction impulse (clamped to |Jt| <= mu * Jn),
//                          followed by a Baumgarte positional-correction pass.
// It builds only on the existing detection layer (collision<A,B>(),
// IntersectData.m_normal, Broadphase::sweepAndPrune) — it does not modify any of
// it. See PhysicsEngineComponent for the Entity/Component bridge.
namespace Physics {

class PhysicsEngine {
   public:
    PhysicsEngine() = default;

    // Copies the body in; returns its stable index (used by
    // PhysicsObjectComponent to read the simulated position back out).
    std::size_t AddObject(const PhysicsObject& object);

    PhysicsObject& GetObject(std::size_t i) { return m_objects[i]; }
    const PhysicsObject& GetObject(std::size_t i) const { return m_objects[i]; }
    std::size_t GetNumObjects() const { return m_objects.size(); }

    void SetGravity(const Vector3f& g) { m_gravity = g; }
    const Vector3f& GetGravity() const { return m_gravity; }

    // Bounciness in [0, 1]: 1 = perfectly elastic (energy preserved), 0 = fully
    // inelastic (no bounce). Default 1.0 reproduces the original reflection
    // behavior; a demo wanting objects to settle should lower it (~0.2-0.4).
    void SetRestitution(float e) { m_restitution = e; }
    float GetRestitution() const { return m_restitution; }

    // CCD auto-threshold, in DISTANCE PER STEP: any body travelling further
    // than this in one fixed step is treated as continuous even without
    // PhysicsObject::SetContinuous(true). Default: infinity (auto-CCD
    // disabled), so CCD is strictly opt-in and default behavior — and the
    // whole existing test suite — is unchanged. A scene protecting itself from
    // fast throws would set roughly its thinnest geometry's half-thickness.
    void SetCcdSpeedThreshold(float distancePerStep) {
        m_ccdSpeedThreshold = distancePerStep;
    }
    float GetCcdSpeedThreshold() const { return m_ccdSpeedThreshold; }

    // Sleeping (SLEEP-1): bodies at rest long enough are put to sleep —
    // skipped by integration and the solver — and wake on contact with an
    // awake body, on WakeUp(), or on an external velocity change. Islands
    // (contact-connected groups of dynamic bodies) sleep and wake as one.
    // MASTER SWITCH DEFAULTS OFF: zero behavior change unless a scene opts in.
    // Disabling wakes everything so no body is stranded asleep.
    void SetSleepingEnabled(bool on);
    bool IsSleepingEnabled() const { return m_sleepingEnabled; }
    // A body sleeps once BOTH its linear and angular speeds have stayed below
    // these for TimeToSleep seconds (and its whole island qualifies too).
    void SetSleepThreshold(float linearSpeed, float angularSpeed) {
        m_sleepLinearThreshold = linearSpeed;
        m_sleepAngularThreshold = angularSpeed;
    }
    void SetTimeToSleep(float seconds) { m_timeToSleep = seconds; }

    void Simulate(float delta);
    void HandleCollisions();

   private:
    // One live contact for the sequential-impulse solver, rebuilt every step
    // from the narrow phase; Pn/Pt1/Pt2 accumulate impulses across iterations
    // (and seed from m_impulseCache for warm starting). A SPECULATIVE contact
    // (CCD) is built for a separated-but-approaching pair: its targetVn is
    // NEGATIVE (-gap/dt — the body may close at most the current gap in one
    // step) and it gets no friction, restitution, or positional correction.
    struct Contact {
        std::size_t ia, ib;         // body indices, normal points ia -> ib
        Vector3f n, t1, t2;         // unit normal + orthonormal tangent basis
        Vector3f rA, rB;            // contact lever arms from each center
        float normalMass{0};        // effective mass along n
        float tangentMass1{0}, tangentMass2{0};
        float targetVn{0};          // desired post-solve normal velocity
        float penetration{0};
        float friction{0};          // combined pair coefficient
        bool speculative{false};    // CCD contact on a not-yet-touching pair
        float Pn{0}, Pt1{0}, Pt2{0};
    };

    // Accumulated impulses remembered per body pair for next-step warm
    // starting (the tangent basis is a deterministic function of the normal,
    // so scalar magnitudes are enough).
    struct CachedImpulse {
        float Pn{0}, Pt1{0}, Pt2{0};
    };

    // Velocity of body `o` at the contact offset r (linear + omega x r).
    static Vector3f ContactVelocity(const PhysicsObject& o, const Vector3f& r);
    // Apply impulse P at lever arm r (linear + angular) with sign `sign`.
    static void ApplyImpulse(PhysicsObject& o, const Vector3f& r,
                             const Vector3f& impulse, float sign);

    bool BuildContact(std::size_t ia, std::size_t ib, const IntersectData& d,
                      bool speculative, Contact& out) const;
    void WarmStart(Contact& c);
    void SolveVelocity(Contact& c);
    void CorrectPositions(const Contact& c);
    // True if this body gets CCD treatment this step (opted in, or moving
    // further per step than the auto-threshold).
    bool CcdActive(const PhysicsObject& o) const;

    // SLEEP-1 passes (no-ops unless sleeping is enabled). WakeIslands: union
    // contact-connected dynamic bodies and wake every island containing an
    // awake member (also drops solver contacts whose endpoints all sleep).
    // SleepIslands: advance per-body rest timers and put each island to sleep
    // once every member has rested past m_timeToSleep.
    void WakeIslands(std::vector<Contact>& contacts);
    void SleepIslands(const std::vector<Contact>& contacts);

    std::vector<PhysicsObject> m_objects;
    Vector3f m_gravity{0.0f, -9.81f, 0.0f};
    float m_restitution{1.0f};
    float m_ccdSpeedThreshold{std::numeric_limits<float>::infinity()};
    bool m_sleepingEnabled{false};       // SLEEP-1 master switch, default OFF
    float m_sleepLinearThreshold{0.05f};   // units/s
    float m_sleepAngularThreshold{0.05f};  // rad/s
    float m_timeToSleep{0.5f};             // seconds of rest before sleeping
    // Duration of the most recent Simulate() step: converts the CCD gap into
    // an allowed approach speed and sizes the swept broad-phase bound. Falls
    // back to 1/60 if HandleCollisions is called before any Simulate.
    float m_lastDelta{1.0f / 60.0f};
    std::map<std::pair<std::size_t, std::size_t>, CachedImpulse> m_impulseCache;
};

}  // namespace Physics
