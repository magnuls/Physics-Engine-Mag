#include "obb.h"

namespace Physics {

OBB::OBB(const Vector3f& center, const Vector3f& halfExtents,
         const Quaternion& orientation)
    : m_center(center),
      m_halfExtents(halfExtents),
      m_orientation(orientation) {}

const Vector3f& OBB::getCenter() const { return m_center; }
const Vector3f& OBB::getHalfExtents() const { return m_halfExtents; }
const Quaternion& OBB::getOrientation() const { return m_orientation; }

Vector3f OBB::axisX() const { return m_orientation.GetRight(); }
Vector3f OBB::axisY() const { return m_orientation.GetUp(); }
Vector3f OBB::axisZ() const { return m_orientation.GetForward(); }

}  // namespace Physics
