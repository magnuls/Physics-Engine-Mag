#include "boundingSphere.h"

#include "../core/math3d.h"

namespace Physics {

BoundingSphere::BoundingSphere(const Vector3f& center, const double& radius)
    : m_center(center), m_radius(radius) {}

// member functions
const Vector3f& BoundingSphere::getCenter() const { return m_center; }
float BoundingSphere::getRadius() const { return m_radius; }

}  // namespace Physics
