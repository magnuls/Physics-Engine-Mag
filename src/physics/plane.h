#pragma once
#include "../core/math3d.h"
// norm is the plane orientation. scaler is its offset in 3D space.

namespace Physics {

class Plane {
   private:
    const Vector3f norm;
    const float scaler;

   public:
    const Vector3f& getNorm() const;
    const float& getScaler() const;

    Plane(const Vector3f& vec, const float& coef);

    Plane normalize() const;
};

}  // namespace Physics
