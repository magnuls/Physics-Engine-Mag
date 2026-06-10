#pragma once
// Essentially this will store the logic for collision detection between any of
// our object types

#include "boundingSphere.h"
#include "intersectData.h"

namespace Physics {
template <typename A, typename B>
IntersectData collision(const A& a, const B& b);

}
