#pragma once
// Collision detection dispatch between shape types.

#include "boundingSphere.h"
#include "intersectData.h"

namespace Physics {
template <typename A, typename B>
IntersectData collision(const A& a, const B& b);

}
