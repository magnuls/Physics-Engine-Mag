#include "colliderComponent.h"

#include <algorithm>

namespace Physics {

// ----------------------------------------------------------------
// SphereCollider
SphereCollider::SphereCollider(float radius, const Vector3f& localCenter)
    : m_localCenter(localCenter), m_localRadius(radius) {}

BoundingSphere SphereCollider::getWorldSphere() const {
    const Transform& t = GetTransform();
    // Full model matrix maps the local center to world space (handles
    // translation, rotation, scale and any parent transforms).
    Vector3f worldCenter =
        Vector3f(t.GetTransformation().Transform(m_localCenter));
    float worldRadius = m_localRadius * t.GetScale();
    return BoundingSphere(worldCenter, worldRadius);
}

// ------------------------------------------------------------------
// AABBCollider
AABBCollider::AABBCollider(const Vector3f& localMin, const Vector3f& localMax)
    : m_localMin(localMin), m_localMax(localMax) {}

AABB AABBCollider::getWorldAABB() const {
    const Transform& t = GetTransform();
    Matrix4f m = t.GetTransformation();

    // Transform all 8 corners of the local box and refit an axis-aligned box
    // around them, so the collider stays a valid AABB even when the entity is
    // rotated (an oriented box is a separate OBB shape, out of scope here).
    const Vector3f& lo = m_localMin;
    const Vector3f& hi = m_localMax;
    const Vector3f corners[8] = {
        Vector3f(lo.GetX(), lo.GetY(), lo.GetZ()),
        Vector3f(hi.GetX(), lo.GetY(), lo.GetZ()),
        Vector3f(lo.GetX(), hi.GetY(), lo.GetZ()),
        Vector3f(lo.GetX(), lo.GetY(), hi.GetZ()),
        Vector3f(hi.GetX(), hi.GetY(), lo.GetZ()),
        Vector3f(hi.GetX(), lo.GetY(), hi.GetZ()),
        Vector3f(lo.GetX(), hi.GetY(), hi.GetZ()),
        Vector3f(hi.GetX(), hi.GetY(), hi.GetZ()),
    };

    Vector3f w = Vector3f(m.Transform(corners[0]));
    Vector3f worldMin = w;
    Vector3f worldMax = w;
    for (int i = 1; i < 8; ++i) {
        w = Vector3f(m.Transform(corners[i]));
        worldMin = Vector3f(std::min(worldMin.GetX(), w.GetX()),
                            std::min(worldMin.GetY(), w.GetY()),
                            std::min(worldMin.GetZ(), w.GetZ()));
        worldMax = Vector3f(std::max(worldMax.GetX(), w.GetX()),
                            std::max(worldMax.GetY(), w.GetY()),
                            std::max(worldMax.GetZ(), w.GetZ()));
    }
    return AABB(worldMin, worldMax);
}

// -----------------------------------------------------------------
// PlaneCollider
PlaneCollider::PlaneCollider(const Vector3f& localNormal, float localScaler)
    : m_localNormal(localNormal), m_localScaler(localScaler) {}

Plane PlaneCollider::getWorldPlane() const {
    const Transform& t = GetTransform();

    // Rotate the normal by the entity's world rotation (directions ignore
    // translation/scale); re-normalize to stay a unit normal.
    Vector3f worldNormal =
        Vector3f(m_localNormal.Rotate(t.GetTransformedRot()).Normalized());

    // The local point closest to the origin lies at normal * scaler; map it to
    // world space and recompute the signed distance for the rotated normal.
    Vector3f localPoint = Vector3f(m_localNormal * m_localScaler);
    Vector3f worldPoint = Vector3f(t.GetTransformation().Transform(localPoint));
    float worldScaler = worldNormal.Dot(worldPoint);

    return Plane(worldNormal, worldScaler);
}

// -------------------------------------------------------------------
// OBBCollider
OBBCollider::OBBCollider(const Vector3f& localHalfExtents,
                         const Vector3f& localCenter)
    : m_localHalfExtents(localHalfExtents), m_localCenter(localCenter) {}

OBB OBBCollider::getWorldOBB() const {
    const Transform& t = GetTransform();
    // Center maps through the full model matrix; half-extents scale uniformly;
    // orientation is the entity's world rotation (so the box actually rotates
    // with the entity instead of being refit axis-aligned like AABBCollider).
    Vector3f worldCenter =
        Vector3f(t.GetTransformation().Transform(m_localCenter));
    Vector3f worldHalfExtents = Vector3f(m_localHalfExtents * t.GetScale());
    return OBB(worldCenter, worldHalfExtents, t.GetTransformedRot());
}

}  // namespace Physics
