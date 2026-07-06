#pragma once

// Collider components: wrap the standalone physics shapes
// (Physics::BoundingSphere, Physics::AABB, Physics::Plane) as EntityComponents so a
// collider is attached to an Entity and follows its Transform. Each component
// stores the shape in the entity's LOCAL space and, on demand, produces the
// corresponding WORLD-space shape by applying the entity's current Transform
// (translation + rotation + scale + parent chain). This replaces the old model
// where colliders were static value types that could not move with an entity.
//
// The shape value types and the collision<>() math are reused unchanged; this
// file only adds the entity/transform wiring on top of them.

#include "../core/entityComponent.h"
#include "../core/math3d.h"
#include "aabb.h"
#include "boundingSphere.h"
#include "obb.h"
#include "plane.h"

namespace Physics {

// Lets a future PhysicsEngine identify a collider's shape without RTTI.
enum class ColliderType { SPHERE, AABB, PLANE, OBB };

// Common base for every collider component.
class ColliderComponent : public EntityComponent {
   public:
    virtual ~ColliderComponent() {}
    virtual ColliderType getType() const = 0;
};

// Sphere collider. Local center defaults to the entity origin.
class SphereCollider : public ColliderComponent {
   public:
    SphereCollider(float radius,
                   const Vector3f& localCenter = Vector3f(0, 0, 0));

    ColliderType getType() const override { return ColliderType::SPHERE; }

    // World-space sphere for the entity's current Transform.
    BoundingSphere getWorldSphere() const;

   private:
    Vector3f m_localCenter;
    float m_localRadius;
};

// Axis-aligned box collider. Stays axis-aligned in world space: under entity
// rotation the box is refit around its transformed corners.
class AABBCollider : public ColliderComponent {
   public:
    AABBCollider(const Vector3f& localMin, const Vector3f& localMax);

    ColliderType getType() const override { return ColliderType::AABB; }

    // World-space AABB for the entity's current Transform.
    AABB getWorldAABB() const;

   private:
    Vector3f m_localMin;
    Vector3f m_localMax;
};

// Plane collider (Hesse form: unit normal + signed distance to origin).
class PlaneCollider : public ColliderComponent {
   public:
    PlaneCollider(const Vector3f& localNormal, float localScaler);

    ColliderType getType() const override { return ColliderType::PLANE; }

    // World-space plane for the entity's current Transform.
    Plane getWorldPlane() const;

   private:
    Vector3f m_localNormal;
    float m_localScaler;
};

// Oriented-box collider. Unlike AABBCollider (which refits to stay axis-aligned)
// this carries the entity's rotation into the shape, so a rotated entity gets a
// true oriented box. Half-extents are along the box's own local axes.
class OBBCollider : public ColliderComponent {
   public:
    OBBCollider(const Vector3f& localHalfExtents,
                const Vector3f& localCenter = Vector3f(0, 0, 0));

    ColliderType getType() const override { return ColliderType::OBB; }

    // World-space OBB for the entity's current Transform (center from position,
    // half-extents scaled, orientation from the entity's world rotation).
    OBB getWorldOBB() const;

   private:
    Vector3f m_localHalfExtents;
    Vector3f m_localCenter;
};

}  // namespace Physics
