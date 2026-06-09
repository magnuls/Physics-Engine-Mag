#pragma once
#include <cmath>

#include "../core/math3d.h"
#include "intersectData.h"

// representation of my object
namespace Physics {
class BoundingSphere {
   private:
    const Vector3f m_center;
    const float m_radius;

   public:
    BoundingSphere(const Vector3f& center, const double& radius);

    // member functions
    const Vector3f& getCenter() const;
    float getRadius() const;
};
}  // namespace Physics
