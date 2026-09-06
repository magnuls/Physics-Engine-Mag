#pragma once
#include "../core/math3d.h"

// Oriented bounding box: a box that can rotate, unlike the always axis aligned
// AABB. Center, half extents along its own local axes, and an orientation quat.
namespace Physics {
class OBB {
   private:
    Vector3f m_center;
    Vector3f m_halfExtents;    // half size along local x, y, z, all > 0
    Quaternion m_orientation;  // assumed unit, identity means axis aligned

   public:
    OBB(const Vector3f& center, const Vector3f& halfExtents,
        const Quaternion& orientation = Quaternion(0, 0, 0, 1));

    const Vector3f& getCenter() const;
    const Vector3f& getHalfExtents() const;
    const Quaternion& getOrientation() const;

    // Unit local axes in world space, the box's own x, y, z directions.
    Vector3f axisX() const;  // local +x right
    Vector3f axisY() const;  // local +y up
    Vector3f axisZ() const;  // local +z forward
};
}  // namespace Physics
