#include "collisionDispatch.h"

#include <algorithm>
#include <cmath>

#include "../src/core/math3d.h"
#include "aabb.h"
#include "boundingSphere.h"
#include "contactPoints.h"
#include "intersectData.h"
#include "plane.h"

using namespace Physics;

namespace {
// Small shared epsilon for degenerate-direction guards.
constexpr float kEps = 1e-6f;

// Center of an AABB.
Vector3f aabbCenter(const AABB& a) { return (a.getMin() + a.getMax()) / 2; }

// Outward normal of the box face nearest an interior point p. Used as the
// contact normal when a sphere center sits inside an AABB.
Vector3f nearestFaceNormal(const AABB& box, const Vector3f& p) {
    const Vector3f mn{box.getMin()};
    const Vector3f mx{box.getMax()};
    const float dist[6]{p.GetX() - mn.GetX(), mx.GetX() - p.GetX(),
                        p.GetY() - mn.GetY(), mx.GetY() - p.GetY(),
                        p.GetZ() - mn.GetZ(), mx.GetZ() - p.GetZ()};
    const Vector3f face[6]{Vector3f(-1, 0, 0), Vector3f(1, 0, 0),
                           Vector3f(0, -1, 0), Vector3f(0, 1, 0),
                           Vector3f(0, 0, -1), Vector3f(0, 0, 1)};
    int best{0};
    for (int i = 1; i < 6; ++i)
        if (dist[i] < dist[best]) best = i;
    return face[best];
}
}  // namespace

template <>
IntersectData Physics::collision<BoundingSphere, BoundingSphere>(
    const BoundingSphere& A, const BoundingSphere& B) {
    Vector3f delta{B.getCenter() - A.getCenter()};  // A toward B
    float dV{delta.Length()};
    float radius_distance{A.getRadius() + B.getRadius()};

    Vector3f normal{dV > kEps ? delta / dV : Vector3f(1, 0, 0)};
    // Point on A's surface facing B.
    Vector3f contact{A.getCenter() + normal * A.getRadius()};

    // the = is when the two spheres are touching
    return IntersectData(dV <= radius_distance, dV - radius_distance, normal,
                         contact);
}

// Overlap iff they overlap on every axis, x,y,z:
template <>
IntersectData Physics::collision<AABB, AABB>(const AABB& A, const AABB& B) {
    Vector3f d1{B.getMin() - A.getMax()};
    Vector3f d2{A.getMin() - B.getMax()};
    Vector3f max{d1.Max(d2)};
    bool hit{std::ranges::all_of(max.cbegin(), max.cend(),
                                 [&](const float& numb) { return numb <= 0; })};

    // Min-penetration axis = the component of max closest to zero. The normal
    // lies along it, oriented from A's center toward B's center.
    int axis{0};
    for (int i = 1; i < 3; ++i)
        if (max[i] > max[axis]) axis = i;

    Vector3f aC{aabbCenter(A)};
    Vector3f bC{aabbCenter(B)};
    Vector3f normal{0, 0, 0};
    normal[axis] = (bC[axis] - aC[axis]) >= 0 ? 1.0f : -1.0f;

    // Contact point = center of the overlap region.
    Vector3f oMin(std::max(A.getMin().GetX(), B.getMin().GetX()),
                  std::max(A.getMin().GetY(), B.getMin().GetY()),
                  std::max(A.getMin().GetZ(), B.getMin().GetZ()));
    Vector3f oMax(std::min(A.getMax().GetX(), B.getMax().GetX()),
                  std::min(A.getMax().GetY(), B.getMax().GetY()),
                  std::min(A.getMax().GetZ(), B.getMax().GetZ()));
    Vector3f contact{(oMin + oMax) / 2};

    return IntersectData(hit, max.Length(), normal, contact);
}

template <>
IntersectData Physics::collision<AABB, BoundingSphere>(
    const AABB& A, const BoundingSphere& B) {
    Vector3f closest(
        std::clamp(B.getCenter().GetX(), A.getMin().GetX(), A.getMax().GetX()),
        std::clamp(B.getCenter().GetY(), A.getMin().GetY(), A.getMax().GetY()),
        std::clamp(B.getCenter().GetZ(), A.getMin().GetZ(), A.getMax().GetZ()));

    Vector3f diff{closest - B.getCenter()};
    float length{diff.Length()};

    // Normal points from box toward sphere. If the sphere center is inside the
    // box, fall back to the nearest-face normal.
    Vector3f normal{length > kEps ? (B.getCenter() - closest) / length
                                  : nearestFaceNormal(A, B.getCenter())};

    return IntersectData(length <= B.getRadius(), length, normal, closest);
}

template <>
IntersectData Physics::collision<BoundingSphere, AABB>(const BoundingSphere& B,
                                                       const AABB& A) {
    IntersectData inter{Physics::collision(A, B)};
    inter.m_normal = inter.m_normal * -1.0f;  // flip to point from sphere to box
    return inter;
}

template <>
IntersectData Physics::collision<BoundingSphere, Plane>(const BoundingSphere& B,
                                                        const Plane& P) {
    Plane normalized{P.normalize()};
    float signedDist{normalized.getNorm().Dot(B.getCenter()) -
                     normalized.getScaler()};
    float dist{std::abs(signedDist)};

    // From sphere toward plane, opposite the side the center is on.
    // Contact = sphere center projected onto the plane.
    Vector3f normal{normalized.getNorm() * (signedDist >= 0 ? -1.0f : 1.0f)};
    Vector3f contact{B.getCenter() - normalized.getNorm() * signedDist};

    return IntersectData(dist <= B.getRadius(), dist, normal, contact);
}

template <>
IntersectData Physics::collision<Plane, BoundingSphere>(
    const Plane& P, const BoundingSphere& B) {
    IntersectData inter{collision(B, P)};
    inter.m_normal = inter.m_normal * -1.0f;  // flip to point from plane to sphere
    return inter;
}

template <>
IntersectData Physics::collision<Plane, Plane>(const Plane& P1,
                                               const Plane& P2) {
    Plane n1{P1.normalize()}, n2{P2.normalize()};
    Vector3f cross{n1.getNorm().Cross(n2.getNorm())};
    IntersectData inter(true, 0);
    if (cross.Length() < kEps) {
        // Parallel: no line of intersection; report gap along the normal.
        inter.m_doesIntersect = false;
        inter.distance = std::abs(n1.getScaler() - n2.getScaler());
        inter.m_normal = n1.getNorm();
    } else {
        // Intersecting planes meet along a line; its direction is the normal.
        inter.m_normal = cross.Normalized();
    }
    return inter;
}

template <>
IntersectData Physics::collision<Plane, AABB>(const Plane& P, const AABB& A) {
    // Normalized norm lmao
    Plane normalized{P.normalize()};
    Vector3f norm_norm{normalized.getNorm()};
    Vector3f A_center{aabbCenter(A)};
    float radius{((A.getMax().GetX() - A.getMin().GetX()) / 2) *
                     std::abs(norm_norm.GetX()) +
                 ((A.getMax().GetY() - A.getMin().GetY()) / 2) *
                     std::abs(norm_norm.GetY()) +
                 ((A.getMax().GetZ() - A.getMin().GetZ()) / 2) *
                     std::abs(norm_norm.GetZ())};
    float signedDist{norm_norm.Dot(A_center) - normalized.getScaler()};
    float distance{std::abs(signedDist)};

    // From plane toward box, the side the box center is on.
    Vector3f normal{norm_norm * (signedDist >= 0 ? 1.0f : -1.0f)};
    // Contact = the box's support point toward the plane, so it sits on the box
    // surface. toPlane points from the box into the plane.
    Vector3f toPlane{norm_norm * (signedDist >= 0 ? -1.0f : 1.0f)};
    Vector3f contact{aabbSupportPoint(A, toPlane)};

    return IntersectData(distance <= radius, distance, normal, contact);
}

template <>
IntersectData Physics::collision<AABB, Plane>(const AABB& A, const Plane& P) {
    IntersectData inter{collision(P, A)};
    inter.m_normal = inter.m_normal * -1.0f;  // flip to point from box to plane
    return inter;
}
