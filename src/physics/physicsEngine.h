#pragma once
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "../core/math3d.h"
#include "intersectData.h"
#include "physicsObject.h"

// The physics simulation loop. PhysicsEngine owns a list of PhysicsObjects.
// Each step integrates them, then solves contacts with a sequential-impulse solver.
namespace Physics {

class PhysicsEngine {
   public:
    PhysicsEngine() = default;

    // Copies the body in and returns its stable index.
    std::size_t AddObject(const PhysicsObject& object);

    PhysicsObject& GetObject(std::size_t i) { return m_objects[i]; }
    const PhysicsObject& GetObject(std::size_t i) const { return m_objects[i]; }
    std::size_t GetNumObjects() const { return m_objects.size(); }

    void SetGravity(const Vector3f& g) { m_gravity = g; }
    const Vector3f& GetGravity() const { return m_gravity; }

    // Bounciness in [0,1]. 1 is elastic, 0 is fully inelastic. Default 1.0.
    void SetRestitution(float e) { m_restitution = e; }
    float GetRestitution() const { return m_restitution; }

    // Bodies travelling further than this per step are treated as continuous.
    // Default infinity means auto-CCD is off, so CCD stays opt-in.
    void SetCcdSpeedThreshold(float distancePerStep) {
        m_ccdSpeedThreshold = distancePerStep;
    }
    float GetCcdSpeedThreshold() const { return m_ccdSpeedThreshold; }

    // Bodies at rest long enough sleep and wake on contact, WakeUp, or a
    // velocity change. Islands sleep and wake as one. Default off; disabling
    // wakes everything.
    void SetSleepingEnabled(bool on);
    bool IsSleepingEnabled() const { return m_sleepingEnabled; }
    // A body sleeps once its linear and angular speeds stay below these for
    // TimeToSleep seconds.
    void SetSleepThreshold(float linearSpeed, float angularSpeed) {
        m_sleepLinearThreshold = linearSpeed;
        m_sleepAngularThreshold = angularSpeed;
    }
    void SetTimeToSleep(float seconds) { m_timeToSleep = seconds; }

    void Simulate(float delta);
    void HandleCollisions();

   private:
    // One contact for the solver, rebuilt each step; Pn/Pt1/Pt2 accumulate
    // impulses. A speculative contact is a not-yet-touching CCD pair with no
    // friction, restitution or positional correction.
    struct Contact {
        std::size_t ia, ib;   // body indices, normal points ia -> ib
        Vector3f n, t1, t2;   // unit normal + orthonormal tangent basis
        Vector3f rA, rB;      // contact lever arms from each center
        float normalMass{0};  // effective mass along n
        float tangentMass1{0}, tangentMass2{0};
        float targetVn{0};  // desired post-solve normal velocity
        float penetration{0};
        float friction{0};        // combined pair coefficient
        bool speculative{false};  // CCD contact on a not-yet-touching pair
        float Pn{0}, Pt1{0}, Pt2{0};
    };

    // Accumulated impulses cached per body pair for next-step warm starting.
    struct CachedImpulse {
        float Pn{0}, Pt1{0}, Pt2{0};
    };

    // Velocity of body o at contact offset r, linear plus omega x r.
    static Vector3f ContactVelocity(const PhysicsObject& o, const Vector3f& r);
    // Apply impulse at lever arm r, both linear and angular.
    static void ApplyImpulse(PhysicsObject& o, const Vector3f& r,
                             const Vector3f& impulse, float sign);

    bool BuildContact(std::size_t ia, std::size_t ib, const IntersectData& d,
                      bool speculative, Contact& out) const;
    void WarmStart(Contact& c);
    void SolveVelocity(Contact& c);
    void CorrectPositions(const Contact& c);
    // True if this body gets CCD this step, opted in or above the threshold.
    bool CcdActive(const PhysicsObject& o) const;

    // Sleep passes, no-ops unless sleeping is on. WakeIslands wakes any island
    // with an awake member; SleepIslands sleeps an island once every member has
    // rested.
    void WakeIslands(std::vector<Contact>& contacts);
    void SleepIslands(const std::vector<Contact>& contacts);

    std::vector<PhysicsObject> m_objects;
    Vector3f m_gravity{0.0f, -9.81f, 0.0f};
    float m_restitution{1.0f};
    float m_ccdSpeedThreshold{std::numeric_limits<float>::infinity()};
    bool m_sleepingEnabled{false};         // master switch, default off
    float m_sleepLinearThreshold{0.05f};   // units/s
    float m_sleepAngularThreshold{0.05f};  // rad/s
    float m_timeToSleep{0.5f};             // seconds of rest before sleeping
    // Duration of the last Simulate step. Sizes the CCD approach speed and
    // swept bounds. Falls back to 1/60 if HandleCollisions runs before Simulate.
    float m_lastDelta{1.0f / 60.0f};
    std::map<std::pair<std::size_t, std::size_t>, CachedImpulse> m_impulseCache;
};

}  // namespace Physics
