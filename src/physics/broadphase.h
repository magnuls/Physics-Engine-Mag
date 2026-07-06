#pragma once
#include <utility>
#include <vector>

#include "aabb.h"

// Broad-phase collision culling.
//
// The narrow-phase collision<A, B>() dispatch is a brute-force O(N^2) test over
// every pair of shapes. The broad-phase cheaply culls that down to a small set
// of candidate pairs whose axis-aligned bounds actually overlap; only those
// candidates need the (more expensive) exact narrow-phase test. Callers wrap
// each shape in its AABB bound, run sweepAndPrune(), then call collision() on
// the returned index pairs.
namespace Physics {

class Broadphase {
   public:
    // True iff two AABBs overlap on all three axes (touching counts).
    static bool overlaps(const AABB& a, const AABB& b);

    // Reference all-pairs test, O(N^2). Ground truth for sweepAndPrune().
    // Returns overlapping index pairs (i, j) with i < j into `boxes`.
    static std::vector<std::pair<int, int>> bruteForcePairs(
        const std::vector<AABB>& boxes);

    // Sweep-and-prune: sort boxes by their min-x endpoint, sweep left to right
    // keeping an "active" set of boxes whose x-interval is still open, and only
    // full-test the current box against that active set. O(N log N + K) for K
    // reported pairs. Returns overlapping index pairs (i, j) with i < j.
    static std::vector<std::pair<int, int>> sweepAndPrune(
        const std::vector<AABB>& boxes);
};

}  // namespace Physics
