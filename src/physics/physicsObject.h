#pragma once
#include <cstddef>
#include <variant>

#include "../core/math3d.h"
#include "aabb.h"
#include "boundingSphere.h"
#include "obb.h"
#include "plane.h"

// A single rigid body: shape params plus position and velocity, building its
// world-space collider on demand. Semi-implicit Euler; inverse mass 0 is static.
namespace Physics {

// Runtime shape tag, separate from ColliderType.
enum class ShapeKind { SPHERE, BOX, PLANE, OBB };

// Collider variant fed to the narrow-phase collision dispatch.
using WorldShape = std::variant<BoundingSphere, AABB, Plane, OBB>;

class PhysicsObject {
   public:
    static PhysicsObject Sphere(const Vector3f& center, float radius,
                                const Vector3f& velocity = Vector3f(0, 0, 0),
                                float invMass = 1.0f);

    static PhysicsObject Box(const Vector3f& min, const Vector3f& max,
                             const Vector3f& velocity = Vector3f(0, 0, 0),
                             float invMass = 1.0f);

    // Static plane in Hesse form. friction is its Coulomb coefficient; a
    // 0-friction floor zeroes all ground friction, since pairs combine as
    // sqrt(muA*muB).
    static PhysicsObject StaticPlane(const Vector3f& normal, float scaler,
                                     float friction = 0.0f);

    // Dynamic oriented box with fixed orientation; integrates position only.
    static PhysicsObject OrientedBox(
        const Vector3f& center, const Vector3f& halfExtents,
        const Quaternion& orientation,
        const Vector3f& velocity = Vector3f(0, 0, 0), float invMass = 1.0f);

    // Advance one fixed step: v += g*dt; pos += v*dt. No-op if static.
    void Integrate(float delta, const Vector3f& gravity);

    // World-space collider for the narrow phase.
    WorldShape GetWorldShape() const;
    // World-space bound for the broad phase.
    AABB GetWorldAABB() const;

    const Vector3f& GetPosition() const { return m_position; }
    void SetPosition(const Vector3f& p) { m_position = p; }
    const Vector3f& GetVelocity() const { return m_velocity; }
    // Setting velocity wakes a sleeping body. The solver writes velocities
    // directly instead, so resting-contact impulses don't reset the sleep timer.
    void SetVelocity(const Vector3f& v) {
        m_velocity = v;
        WakeUp();
    }
    float GetInvMass() const { return m_invMass; }
    // Coulomb friction coefficient, default 0. Pairs combine as sqrt(muA*muB).
    float GetFriction() const { return m_friction; }
    void SetFriction(float mu) { m_friction = mu; }

    // Per-body drag in Integrate, form v *= 1/(1 + d*dt). Angular damping acts
    // as rolling resistance so spinning bodies come to rest. Default 0.
    void SetLinearDamping(float d) { m_linearDamping = d; }
    float GetLinearDamping() const { return m_linearDamping; }
    void SetAngularDamping(float d) { m_angularDamping = d; }
    float GetAngularDamping() const { return m_angularDamping; }

    bool IsStatic() const { return m_invMass == 0.0f; }
    bool IsPlane() const { return m_kind == ShapeKind::PLANE; }

    // CCD opt-in. A continuous body sweeps its broad-phase bound forward so a
    // fast mover is stopped before it tunnels thin geometry. Default off.
    void SetContinuous(bool on) { m_continuous = on; }
    bool IsContinuous() const { return m_continuous; }
    ShapeKind GetShapeKind() const { return m_kind; }

    // A sleeping body is skipped by integration and the solver until woken.
    // Only the engine sleeps a body, and only when sleeping is enabled.
    bool IsAwake() const { return m_awake; }
    void WakeUp() {
        m_awake = true;
        m_sleepTime = 0.0f;
    }

    const Quaternion& GetOrientation() const { return m_orientation; }
    const Vector3f& GetAngularVelocity() const { return m_angularVelocity; }
    void SetAngularVelocity(const Vector3f& w) {
        m_angularVelocity = w;
        WakeUp();  // external change wakes a sleeping body
    }

    // Apply the world inverse inertia tensor to a vector such as a torque.
    // Zero for static bodies, axis-aligned BOX and planes, so they never spin.
    Vector3f ApplyInverseInertia(const Vector3f& worldVec) const;

    // Half-width of the shape projected onto unit direction n.
    // Used to recover penetration depth; planes return 0.
    float SupportRadius(const Vector3f& n) const;

   private:
    // The engine owns the sleep and island bookkeeping and writes velocities
    // and sleep state directly through this friend, bypassing the waking setters.
    friend class PhysicsEngine;

    // Fills m_invInertiaLocal from the shape, dimensions and inverse mass.
    void InitInertia();

    ShapeKind m_kind{ShapeKind::SPHERE};
    Vector3f m_position{0, 0, 0};  // sphere/box center; plane: unused
    Vector3f m_velocity{0, 0, 0};
    float m_invMass{1.0f};     // 0 => static/immovable
    float m_friction{0.0f};    // Coulomb coefficient, 0 = frictionless
    float m_linearDamping{0.0f};   // 1/s, 0 = no drag
    float m_angularDamping{0.0f};  // 1/s, 0 = no drag
    bool m_continuous{false};  // CCD opt-in
    bool m_awake{true};        // false = engine put this body to sleep
    float m_sleepTime{0.0f};   // seconds spent below sleep thresholds

    float m_radius{0.0f};                  // SPHERE
    Vector3f m_halfExtents{0, 0, 0};       // BOX, OBB
    Vector3f m_planeNormal{0, 1, 0};       // PLANE
    float m_planeScaler{0.0f};             // PLANE
    Quaternion m_orientation{0, 0, 0, 1};  // OBB and integrated rotation

    Vector3f m_angularVelocity{0, 0, 0};  // rad/s, world space
    // Diagonal of the local inverse inertia tensor, principal axes.
    // Zero for static bodies, planes and axis-aligned BOX.
    Vector3f m_invInertiaLocal{0, 0, 0};
};

}  // namespace Physics
