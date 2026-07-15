#include "obbCollision.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "contactPoints.h"

// OBB narrow-phase collision, kept out of Agent 3's collisionDispatch.cpp.
// Every result follows the shared convention: m_normal points from the first
// shape (A) toward the second (B), unit on a hit; `distance` is the separation
// gap on a miss and the penetration depth on a hit.
namespace Physics {
namespace {

constexpr float kEps = 1e-6f;

// An OBB unpacked into center, unit axes and half-extents for fast math.
struct Frame {
    Vector3f c;
    Vector3f axis[3];
    float e[3];
};

Frame unpack(const OBB& o) {
    Frame f;
    f.c = o.getCenter();
    f.axis[0] = o.axisX();
    f.axis[1] = o.axisY();
    f.axis[2] = o.axisZ();
    f.e[0] = o.getHalfExtents().GetX();
    f.e[1] = o.getHalfExtents().GetY();
    f.e[2] = o.getHalfExtents().GetZ();
    return f;
}

// An AABB is an OBB with identity orientation (axis-aligned).
OBB fromAABB(const AABB& b) {
    Vector3f center{(b.getMin() + b.getMax()) / 2.0f};
    Vector3f half{(b.getMax() - b.getMin()) / 2.0f};
    return OBB(center, half, Quaternion(0, 0, 0, 1));
}

// Projection radius of a frame onto a unit axis n.
float radius(const Frame& f, const Vector3f& n) {
    return f.e[0] * std::abs(f.axis[0].Dot(n)) +
           f.e[1] * std::abs(f.axis[1].Dot(n)) +
           f.e[2] * std::abs(f.axis[2].Dot(n));
}

// Separating Axis Theorem for two OBBs. Scans all 15 candidate axes (2x3 face
// normals + 9 edge-edge cross products) and keeps the axis of MAXIMUM
// separation. Disjoint iff that maximum is positive. That same axis gives the
// contact normal and, negated, the penetration depth.
IntersectData satObbObb(const OBB& A, const OBB& B) {
    const Frame a{unpack(A)};
    const Frame b{unpack(B)};
    const Vector3f t{b.c - a.c};  // A -> B

    Vector3f candidates[15];
    candidates[0] = a.axis[0];
    candidates[1] = a.axis[1];
    candidates[2] = a.axis[2];
    candidates[3] = b.axis[0];
    candidates[4] = b.axis[1];
    candidates[5] = b.axis[2];
    int k = 6;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            candidates[k++] = a.axis[i].Cross(b.axis[j]);

    float bestSep = -std::numeric_limits<float>::infinity();
    Vector3f bestAxis{a.axis[0]};

    for (int i = 0; i < 15; ++i) {
        float len = candidates[i].Length();
        if (len < kEps) continue;  // degenerate (parallel edges) — not an axis
        Vector3f n{candidates[i] / len};
        float sep = std::abs(t.Dot(n)) - (radius(a, n) + radius(b, n));
        if (sep > bestSep) {
            bestSep = sep;
            bestAxis = n;
        }
    }

    // Orient the chosen axis from A toward B.
    if (t.Dot(bestAxis) < 0.0f) bestAxis = bestAxis * -1.0f;

    bool hit = bestSep <= 0.0f;
    // Contact point ON the touching surfaces (A2, Agent 3's obbSupportPoint):
    // midpoint of A's furthest point toward B and B's furthest point toward A
    // along the contact axis. For a face-face contact the zero-dot axes fall
    // back to face centers, so a resting box gets a stable centered contact;
    // an edge/corner contact returns the actual corner — that off-center lever
    // arm is what makes a box landing on a corner tip (A1's angular response).
    Vector3f pa{obbSupportPoint(a.c, Vector3f(a.e[0], a.e[1], a.e[2]),
                                a.axis[0], a.axis[1], a.axis[2], bestAxis)};
    Vector3f pb{obbSupportPoint(b.c, Vector3f(b.e[0], b.e[1], b.e[2]),
                                b.axis[0], b.axis[1], b.axis[2],
                                bestAxis * -1.0f)};
    Vector3f contact{(pa + pb) / 2.0f};
    return IntersectData(hit, std::abs(bestSep), bestAxis, contact);
}

}  // namespace

template <>
IntersectData collision<OBB, OBB>(const OBB& a, const OBB& b) {
    return satObbObb(a, b);
}

template <>
IntersectData collision<OBB, AABB>(const OBB& a, const AABB& b) {
    return satObbObb(a, fromAABB(b));
}

template <>
IntersectData collision<AABB, OBB>(const AABB& a, const OBB& b) {
    IntersectData inter{satObbObb(fromAABB(a), b)};
    return inter;  // A=box(as OBB) -> B=obb already correct
}

template <>
IntersectData collision<OBB, BoundingSphere>(const OBB& a,
                                             const BoundingSphere& b) {
    const Frame f{unpack(a)};
    Vector3f d{b.getCenter() - f.c};
    float l[3]{d.Dot(f.axis[0]), d.Dot(f.axis[1]), d.Dot(f.axis[2])};
    float cl[3]{std::clamp(l[0], -f.e[0], f.e[0]),
                std::clamp(l[1], -f.e[1], f.e[1]),
                std::clamp(l[2], -f.e[2], f.e[2])};

    Vector3f closest{f.c + f.axis[0] * cl[0] + f.axis[1] * cl[1] +
                     f.axis[2] * cl[2]};
    Vector3f diff{b.getCenter() - closest};
    float len{diff.Length()};

    // Normal points from box (A) toward sphere (B). When the sphere center is
    // inside the box, fall back to the nearest local face's outward normal.
    Vector3f normal;
    if (len > kEps) {
        normal = diff / len;
    } else {
        float toFace[6]{f.e[0] - l[0], f.e[0] + l[0], f.e[1] - l[1],
                        f.e[1] + l[1], f.e[2] - l[2], f.e[2] + l[2]};
        Vector3f faces[6]{f.axis[0], f.axis[0] * -1.0f,
                          f.axis[1], f.axis[1] * -1.0f,
                          f.axis[2], f.axis[2] * -1.0f};
        int best{0};
        for (int i = 1; i < 6; ++i)
            if (toFace[i] < toFace[best]) best = i;
        normal = faces[best];
    }
    return IntersectData(len <= b.getRadius(), len, normal, closest);
}

template <>
IntersectData collision<BoundingSphere, OBB>(const BoundingSphere& a,
                                             const OBB& b) {
    IntersectData inter{collision<OBB, BoundingSphere>(b, a)};
    inter.m_normal = inter.m_normal * -1.0f;  // flip to sphere -> box
    return inter;
}

template <>
IntersectData collision<OBB, Plane>(const OBB& a, const Plane& b) {
    const Frame f{unpack(a)};
    Plane normalized{b.normalize()};
    Vector3f n{normalized.getNorm()};
    float r{radius(f, n)};
    float signedDist{n.Dot(f.c) - normalized.getScaler()};

    // From the box (A) toward the plane (B): opposite the side the center is
    // on.
    Vector3f normal{n * (signedDist >= 0 ? -1.0f : 1.0f)};
    // Contact = the box point deepest toward the plane (A2 support point), not
    // the center's projection: a tilted box contacts the floor at its corner,
    // giving the off-center lever arm that lets it tip over and settle flat.
    // A face-flat box still gets its face center (zero-dot axes fall back).
    Vector3f contact{obbSupportPoint(f.c, a.getHalfExtents(), f.axis[0],
                                     f.axis[1], f.axis[2], normal)};
    return IntersectData(std::abs(signedDist) <= r, std::abs(signedDist),
                         normal, contact);
}

template <>
IntersectData collision<Plane, OBB>(const Plane& a, const OBB& b) {
    IntersectData inter{collision<OBB, Plane>(b, a)};
    inter.m_normal = inter.m_normal * -1.0f;  // flip to plane -> box
    return inter;
}

}  // namespace Physics
