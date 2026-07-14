#pragma once
#include "../core/math3d.h"

// Result of a collision query: intersect flag, distance, and a contact manifold.
// distance is separation on a miss or penetration depth on a hit.
namespace Physics {
struct IntersectData {
    float distance;
    bool m_doesIntersect;
    Vector3f m_normal;        // unit contact normal, from A toward B
    Vector3f m_contactPoint;  // representative world contact point

    IntersectData(bool doesIntersect, float dist)
        : distance(dist),
          m_doesIntersect(doesIntersect),
          m_normal(0, 0, 0),
          m_contactPoint(0, 0, 0) {};

    IntersectData(bool doesIntersect, float dist, const Vector3f& normal,
                  const Vector3f& contactPoint)
        : distance(dist),
          m_doesIntersect(doesIntersect),
          m_normal(normal),
          m_contactPoint(contactPoint) {};
};
}  // namespace Physics
