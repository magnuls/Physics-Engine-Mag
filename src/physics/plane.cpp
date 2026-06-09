#include "plane.h"

using namespace Physics;

Plane::Plane(const Vector3f& vec, const float& coef)
    : norm(vec), scaler(coef) {};

const Vector3f& Plane::getNorm() const { return norm; }
const float& Plane::getScaler() const { return scaler; }

Plane Plane::normalize() const {
    Plane normal(norm.Normalized(), scaler / norm.Length());
    return normal;
}
