#pragma once
#include "../core/math3d.h"
#include "aabb.h"

// Contact-point helpers. The contact point must sit on the contact surface so the
// lever arm r = contactPoint - centerOfMass gives correct torque.
namespace Physics {

// Support point of an axis-aligned box: the point furthest along dir. A
// perpendicular axis returns the face center, keeping flat contacts stable.
inline Vector3f aabbSupportPoint(const AABB& box, const Vector3f& dir) {
    const Vector3f mn{box.getMin()};
    const Vector3f mx{box.getMax()};
    auto pick = [](float d, float lo, float hi) {
        if (d > 0.0f) return hi;
        if (d < 0.0f) return lo;
        return 0.5f * (lo + hi);
    };
    return Vector3f(pick(dir.GetX(), mn.GetX(), mx.GetX()),
                    pick(dir.GetY(), mn.GetY(), mx.GetY()),
                    pick(dir.GetZ(), mn.GetZ(), mx.GetZ()));
}

// Support point of an oriented box: the vertex furthest along dir. A perpendicular
// axis stays centered. For an OBB vs plane, pass the outward normal negated as dir.
inline Vector3f obbSupportPoint(const Vector3f& center,
                                const Vector3f& halfExtents,
                                const Vector3f& axisX, const Vector3f& axisY,
                                const Vector3f& axisZ, const Vector3f& dir) {
    const Vector3f axis[3]{axisX, axisY, axisZ};
    const float h[3]{halfExtents.GetX(), halfExtents.GetY(),
                     halfExtents.GetZ()};
    Vector3f p{center};
    for (int i = 0; i < 3; ++i) {
        const float d = dir.Dot(axis[i]);
        const float s = d > 1e-6f ? 1.0f : (d < -1e-6f ? -1.0f : 0.0f);
        p += axis[i] * (s * h[i]);
    }
    return p;
}

}  // namespace Physics
