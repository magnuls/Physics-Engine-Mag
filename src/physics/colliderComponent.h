#pragma once

// Collider components wrap the physics shapes as EntityComponents so a collider
// follows its entity's Transform, producing a world shape on demand.

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

    // World sphere for the entity's current Transform.
    BoundingSphere getWorldSphere() const;

   private:
    Vector3f m_localCenter;
    float m_localRadius;
};

// Axis aligned box collider. Under entity rotation it refits around the
// transformed corners.
class AABBCollider : public ColliderComponent {
   public:
    AABBCollider(const Vector3f& localMin, const Vector3f& localMax);

    ColliderType getType() const override { return ColliderType::AABB; }

    // World AABB for the entity's current Transform.
    AABB getWorldAABB() const;

   private:
    Vector3f m_localMin;
    Vector3f m_localMax;
};

// Plane collider in Hesse form: unit normal plus signed distance to origin.
class PlaneCollider : public ColliderComponent {
   public:
    PlaneCollider(const Vector3f& localNormal, float localScaler);

    ColliderType getType() const override { return ColliderType::PLANE; }

    // World plane for the entity's current Transform.
    Plane getWorldPlane() const;

   private:
    Vector3f m_localNormal;
    float m_localScaler;
};

// Oriented box collider. Carries the entity's rotation into the shape, so a
// rotated entity gets a true oriented box.
class OBBCollider : public ColliderComponent {
   public:
    OBBCollider(const Vector3f& localHalfExtents,
                const Vector3f& localCenter = Vector3f(0, 0, 0));

    ColliderType getType() const override { return ColliderType::OBB; }

    // World OBB for the entity's current Transform.
    OBB getWorldOBB() const;

   private:
    Vector3f m_localHalfExtents;
    Vector3f m_localCenter;
};

}  // namespace Physics
