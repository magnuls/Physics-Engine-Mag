#pragma once
#include "../core/math3d.h"

// Holds the data if we have intersected and distance between the objects.
//
// Extended with a contact manifold: a unit collision normal and a
// representative world-space contact point, so collision RESPONSE (pushing
// objects apart / applying impulses) can be built on top of detection.
//
// Convention: m_normal points from the FIRST shape (A) toward the SECOND (B)
// in a collision<A, B>(a, b) query, and has unit length when m_doesIntersect.
// distance keeps its original meaning per pair (separation on a miss,
// penetration depth on a hit); the legacy 2-arg ctor is preserved so existing
// callers/tests are unaffected (normal/contact default to zero).
namespace Physics {
struct IntersectData {
    float distance;
    bool m_doesIntersect;
    Vector3f m_normal;        // unit contact normal, from A toward B
    Vector3f m_contactPoint;  // representative world-space contact point

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
