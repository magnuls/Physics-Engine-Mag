#pragma once
#include "../core/math3d.h"
/*
 * The norm is the orientation of our plane
 * The scaler will show us where in 3D space our plane is
 */

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
