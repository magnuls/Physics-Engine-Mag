#pragma once
#include "aabb.h"
#include "boundingSphere.h"
#include "collisionDispatch.h"
#include "obb.h"
#include "plane.h"

// Declarations of the OBB narrow-phase specializations of Physics::collision<>.
// The primary template lives in collisionDispatch.h (owned by Agent 3); these
// OBB overloads are DEFINED in obbCollision.cpp — kept in separate files so the
// existing detection code is never edited. Declaring them here (before use)
// keeps callers that dispatch to collision<OBB, X>() well-formed.
//
// Normal convention (same as every other pair): m_normal points from the FIRST
// shape (A) toward the SECOND (B), unit length on a hit.
namespace Physics {

template <>
IntersectData collision<OBB, OBB>(const OBB& a, const OBB& b);

template <>
IntersectData collision<OBB, BoundingSphere>(const OBB& a,
                                             const BoundingSphere& b);
template <>
IntersectData collision<BoundingSphere, OBB>(const BoundingSphere& a,
                                             const OBB& b);

template <>
IntersectData collision<OBB, AABB>(const OBB& a, const AABB& b);
template <>
IntersectData collision<AABB, OBB>(const AABB& a, const OBB& b);

template <>
IntersectData collision<OBB, Plane>(const OBB& a, const Plane& b);
template <>
IntersectData collision<Plane, OBB>(const Plane& a, const OBB& b);

}  // namespace Physics
