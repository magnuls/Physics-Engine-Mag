#pragma once
#include "../core/math3d.h"
#include "aabb.h"

// Contact-point helpers (A2 — contact-point quality, Agent 3).
//
// A believable angular response needs the contact point to sit on the actual
// contact SURFACE, because the torque a contact applies is driven by its lever
// arm r = contactPoint - centerOfMass. A volume-centroid contact gives the wrong
// (often zero) lever arm; a support point on the touching face/edge/vertex gives
// the right one. These helpers return that support point.
namespace Physics {

// Support point of an axis-aligned box: the point on the box furthest along
// `dir`. Per axis: dir_i > 0 -> max_i, dir_i < 0 -> min_i, dir_i ~ 0 -> the face
// center on that axis (so a flat/face contact returns the face center rather
// than an arbitrary corner, which keeps the lever arm stable).
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

// Support point of an ORIENTED box (center + half-extents + three unit axes):
// the vertex furthest along `dir`. Step +h_i along local axis i when the axis
// points with `dir` (dir . axis_i > 0), -h_i when against it, and 0 when
// perpendicular (a face/edge contact stays centered on that axis).
//
// Provided for Agent 1's OBB narrow-phase (obbCollision.cpp), whose contact
// point is a rough midpoint today — Agent 3 does not edit that file, so adopt
// this there if useful. Example: the OBB contact vs a plane with outward normal
// n is obbSupportPoint(center, half, ax, ay, az, -n).
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
