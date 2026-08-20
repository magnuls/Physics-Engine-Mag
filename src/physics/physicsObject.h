#pragma once
#include <cstddef>
#include <variant>

#include "../core/math3d.h"
#include "aabb.h"
#include "boundingSphere.h"
#include "obb.h"
#include "plane.h"

// A single simulated rigid body for the physics engine.
//
// A PhysicsObject owns its own shape (as a small set of parameters) plus a
// position and velocity, and it builds the corresponding *world-space* collider
// on demand. It is deliberately SELF-CONTAINED: it does not depend on the
// Entity/Transform-driven collider components (those are a separate layer). The
// PhysicsObjectComponent bridge copies the simulated position back onto an
// Entity's Transform each frame.
//
// Motion is semi-implicit (symplectic) Euler. A body with inverse mass 0 is
// STATIC: it is never integrated and is never moved by a collision response
// (used for the ground plane, walls, immovable boxes, etc.).
namespace Physics {

// Runtime shape tag (kept independent of Agent 2's Physics::ColliderType so the
// two layers stay decoupled).
enum class ShapeKind { SPHERE, BOX, PLANE, OBB };

// The world-space collider variant fed to the narrow-phase collision<A,B>()
// dispatch. Order matters only for std::visit; every {Sphere,Box,Plane,OBB}
// pair has an explicit collision<>() specialization (OBB ones in obbCollision).
using WorldShape = std::variant<BoundingSphere, AABB, Plane, OBB>;

class PhysicsObject {
   public:
    // A dynamic sphere centered at `center`.
    static PhysicsObject Sphere(const Vector3f& center, float radius,
                                const Vector3f& velocity = Vector3f(0, 0, 0),
                                float invMass = 1.0f);

    // A dynamic axis-aligned box spanning [min, max].
    static PhysicsObject Box(const Vector3f& min, const Vector3f& max,
                             const Vector3f& velocity = Vector3f(0, 0, 0),
                             float invMass = 1.0f);

    // A static (immovable) plane in Hesse form (unit-ish normal + scalar).
    // `friction` is the plane's Coulomb coefficient (SP-FRIC): the ground is
    // the one body every scene has, and forgetting its friction silently
    // zeroes ALL ground friction (pair combine is sqrt(muA*muB)) — taking it
    // at construction makes that hard to miss. Default 0 keeps the engine's
    // neutral defaults and existing call sites unchanged.
    static PhysicsObject StaticPlane(const Vector3f& normal, float scaler,
                                     float friction = 0.0f);

    // A dynamic oriented (rotatable) box. Orientation is fixed for its lifetime
    // — this first pass integrates position only, no angular velocity/torque.
    static PhysicsObject OrientedBox(
        const Vector3f& center, const Vector3f& halfExtents,
        const Quaternion& orientation,
        const Vector3f& velocity = Vector3f(0, 0, 0), float invMass = 1.0f);

    // Advance one fixed step: v += g*dt; pos += v*dt. No-op if static.
    void Integrate(float delta, const Vector3f& gravity);

    // World-space collider at the current position (for the narrow phase).
    WorldShape GetWorldShape() const;
    // Conservative world-space bound (for the broad phase).
    AABB GetWorldAABB() const;

    const Vector3f& GetPosition() const { return m_position; }
    void SetPosition(const Vector3f& p) { m_position = p; }
    const Vector3f& GetVelocity() const { return m_velocity; }
    // External velocity changes WAKE a sleeping body (SLEEP-1 wake trigger).
    // The engine's own solver writes velocities directly instead, so its
    // resting-contact impulses don't keep resetting the sleep timer.
    void SetVelocity(const Vector3f& v) {
        m_velocity = v;
        WakeUp();
    }
    float GetInvMass() const { return m_invMass; }
    // Coulomb friction coefficient (>= 0). Default 0 = frictionless, which
    // preserves the engine's physically-neutral defaults; scenes/tests dial it
    // up. The solver combines a pair's coefficients as sqrt(muA * muB).
    float GetFriction() const { return m_friction; }
    void SetFriction(float mu) { m_friction = mu; }

    // Damping (DAMP-1): per-body drag applied during integration with the
    // unconditionally-stable Bullet form v *= 1/(1 + d*dt). Angular damping is
    // the rolling-resistance stand-in that lets a rolling/spinning body bleed
    // energy, come to rest and finally sleep — without it, angular velocity
    // only ever changes through contact impulses. Defaults 0 = no drag (the
    // engine stays neutral; scenes dial it up, typically angular > linear).
    void SetLinearDamping(float d) { m_linearDamping = d; }
    float GetLinearDamping() const { return m_linearDamping; }
    void SetAngularDamping(float d) { m_angularDamping = d; }
    float GetAngularDamping() const { return m_angularDamping; }

    bool IsStatic() const { return m_invMass == 0.0f; }
    bool IsPlane() const { return m_kind == ShapeKind::PLANE; }

    // --- CCD (continuous collision detection) opt-in ------------------------
    // A continuous body gets SPECULATIVE CONTACTS (see PhysicsEngine): its
    // broad-phase bound is swept forward by one step's travel, and the solver
    // stops it approaching another body by more than the current gap in a
    // single step — so a fast mover cannot tunnel through thin geometry.
    // Default off (zero behavior change unless enabled).
    void SetContinuous(bool on) { m_continuous = on; }
    bool IsContinuous() const { return m_continuous; }
    ShapeKind GetShapeKind() const { return m_kind; }

    // --- Sleeping (SLEEP-1) --------------------------------------------------
    // A body the engine has put to sleep is skipped by integration and the
    // solver until woken (by contact with an awake body, WakeUp(), or an
    // external velocity change). Only the engine ever sets a body asleep, and
    // only when sleeping is enabled on it (PhysicsEngine::SetSleepingEnabled).
    bool IsAwake() const { return m_awake; }
    void WakeUp() {
        m_awake = true;
        m_sleepTime = 0.0f;
    }

    // --- Rotational state (angular dynamics) --------------------------------
    const Quaternion& GetOrientation() const { return m_orientation; }
    const Vector3f& GetAngularVelocity() const { return m_angularVelocity; }
    void SetAngularVelocity(const Vector3f& w) {
        m_angularVelocity = w;
        WakeUp();  // external change wakes a sleeping body (SLEEP-1)
    }

    // Apply this body's inverse inertia tensor to a world-space vector (e.g. a
    // torque r x J). Sphere inertia is isotropic; the OBB tensor rotates with
    // the box. Static bodies (and axis-aligned BOX, which stays unrotated)
    // return the zero vector, so they never spin.
    Vector3f ApplyInverseInertia(const Vector3f& worldVec) const;

    // Half-width of this shape projected onto the unit direction n (its
    // "support radius"): sphere -> radius; box/OBB -> sum of half-extents *
    // |axis . n|. The engine uses this to recover a consistent penetration
    // depth for positional correction, since the detector's `distance` field
    // does not mean penetration uniformly across shape pairs. Planes return 0
    // (handled specially — a plane is static and positionally fixed).
    float SupportRadius(const Vector3f& n) const;

   private:
    // The engine owns the sleep/island bookkeeping (SLEEP-1: internal, not
    // public API): it writes m_awake/m_sleepTime and — when applying solver
    // impulses — m_velocity/m_angularVelocity directly, bypassing the waking
    // public setters.
    friend class PhysicsEngine;

    // Fills m_invInertiaLocal from the shape, dimensions and inverse mass.
    void InitInertia();

    ShapeKind m_kind{ShapeKind::SPHERE};
    Vector3f m_position{0, 0, 0};  // sphere/box center; plane: unused
    Vector3f m_velocity{0, 0, 0};
    float m_invMass{1.0f};     // 0 => static/immovable
    float m_friction{0.0f};    // Coulomb coefficient, 0 = frictionless
    float m_linearDamping{0.0f};   // DAMP-1: 1/s, 0 = no drag
    float m_angularDamping{0.0f};  // DAMP-1: 1/s, 0 = no drag
    bool m_continuous{false};  // CCD opt-in (speculative contacts)
    bool m_awake{true};        // false = engine put this body to sleep
    float m_sleepTime{0.0f};   // seconds spent below sleep thresholds

    float m_radius{0.0f};                  // SPHERE
    Vector3f m_halfExtents{0, 0, 0};       // BOX, OBB
    Vector3f m_planeNormal{0, 1, 0};       // PLANE
    float m_planeScaler{0.0f};             // PLANE
    Quaternion m_orientation{0, 0, 0, 1};  // OBB (and integrated rotation)

    Vector3f m_angularVelocity{0, 0, 0};  // rad/s, world space
    // Diagonal of the LOCAL inverse inertia tensor (principal axes). Zero for
    // static bodies, planes, and axis-aligned BOX (which does not rotate).
    Vector3f m_invInertiaLocal{0, 0, 0};
};

}  // namespace Physics
