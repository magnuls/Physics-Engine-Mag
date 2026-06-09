#include "aabb.h"

#include <stdexcept>

#include "core/math3d.h"

using namespace Physics;

// Constructor
AABB::AABB(const Vector3f& bottom_left, const Vector3f& top_right)
    : min(bottom_left), max(top_right) {
    if (min.GetX() > max.GetX() || min.GetY() > max.GetY() ||
        min.GetZ() > max.GetZ())
        throw std::runtime_error("Invalid Bottom Left/Bottom Right");
};

// Methods
const Vector3f& AABB::getMin() const { return min; }
const Vector3f& AABB::getMax() const { return max; }
