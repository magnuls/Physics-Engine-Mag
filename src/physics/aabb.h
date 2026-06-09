#pragma once
#include "../core/math3d.h"
#include "intersectData.h"

// Axis Algined bounding box

namespace Physics {
class AABB {
   private:
    Vector3f min;
    Vector3f max;

   public:
    AABB(const Vector3f& bottom_left, const Vector3f& top_right);

    const Vector3f& getMin() const;
    const Vector3f& getMax() const;
};
}  // namespace Physics
