#pragma once
#include "../core/math3d.h"

// Oriented Bounding Box: a box that can be rotated (unlike Physics::AABB, which
// is always axis-aligned). Defined by a center, three half-extents measured
// along the box's OWN local axes, and an orientation quaternion. The local axes
// are the quaternion's right / up / forward vectors, so an identity orientation
// makes the OBB behave exactly like an AABB.
namespace Physics {
class OBB {
   private:
    Vector3f m_center;
    Vector3f m_halfExtents;       // half-size along local x, y, z (all > 0)
    Quaternion m_orientation;     // assumed unit; identity => axis-aligned

   public:
    OBB(const Vector3f& center, const Vector3f& halfExtents,
        const Quaternion& orientation = Quaternion(0, 0, 0, 1));

    const Vector3f& getCenter() const;
    const Vector3f& getHalfExtents() const;
    const Quaternion& getOrientation() const;

    // Unit local axes in world space (the box's own x / y / z directions).
    Vector3f axisX() const;  // local +x (right)
    Vector3f axisY() const;  // local +y (up)
    Vector3f axisZ() const;  // local +z (forward)
};
}  // namespace Physics
