#include "broadphase.h"

#include <algorithm>

using namespace Physics;

bool Broadphase::overlaps(const AABB& a, const AABB& b) {
    return a.getMin().GetX() <= b.getMax().GetX() &&
           b.getMin().GetX() <= a.getMax().GetX() &&
           a.getMin().GetY() <= b.getMax().GetY() &&
           b.getMin().GetY() <= a.getMax().GetY() &&
           a.getMin().GetZ() <= b.getMax().GetZ() &&
           b.getMin().GetZ() <= a.getMax().GetZ();
}

std::vector<std::pair<int, int>> Broadphase::bruteForcePairs(
    const std::vector<AABB>& boxes) {
    std::vector<std::pair<int, int>> pairs;
    const int n{static_cast<int>(boxes.size())};
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (overlaps(boxes[i], boxes[j])) pairs.emplace_back(i, j);
    return pairs;
}

std::vector<std::pair<int, int>> Broadphase::sweepAndPrune(
    const std::vector<AABB>& boxes) {
    std::vector<std::pair<int, int>> pairs;
    const int n{static_cast<int>(boxes.size())};

    // Process boxes in ascending order of their min-x endpoint.
    std::vector<int> order(n);
    for (int i = 0; i < n; ++i) order[i] = i;
    std::ranges::sort(order, [&](int a, int b) {
        return boxes[a].getMin().GetX() < boxes[b].getMin().GetX();
    });

    // `active` holds indices whose x-interval has not yet closed relative to
    // the sweep front. Since we scan by ascending min-x, any box whose max-x is
    // below the current box's min-x can never overlap a later box either.
    std::vector<int> active;
    for (int idx : order) {
        const float curMin{boxes[idx].getMin().GetX()};
        std::erase_if(active, [&](int a) {
            return boxes[a].getMax().GetX() < curMin;
        });
        // Every still-active box overlaps on x; confirm the other two axes.
        for (int a : active)
            if (overlaps(boxes[a], boxes[idx])) {
                pairs.emplace_back(std::min(a, idx), std::max(a, idx));
            }
        active.push_back(idx);
    }
    return pairs;
}
