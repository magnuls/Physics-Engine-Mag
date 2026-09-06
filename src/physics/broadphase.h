#pragma once
#include <utility>
#include <vector>

#include "aabb.h"

// Broadphase collision culling. Cheaply reduces the O(N^2) narrowphase pairs
// to candidate AABB overlaps that then need the exact test.
namespace Physics {

class Broadphase {
   public:
    // True if two AABBs overlap on all three axes. Touching counts.
    static bool overlaps(const AABB& a, const AABB& b);

    // Reference all pairs test, O(N^2), the ground truth for sweepAndPrune.
    // Returns overlapping index pairs i,j with i < j into boxes.
    static std::vector<std::pair<int, int>> bruteForcePairs(
        const std::vector<AABB>& boxes);

    // Sort by min x, sweep an active set of open x intervals, and full test
    // only against it. O(N log N + K), returns pairs i,j with i < j.
    static std::vector<std::pair<int, int>> sweepAndPrune(
        const std::vector<AABB>& boxes);
};

}  // namespace Physics
