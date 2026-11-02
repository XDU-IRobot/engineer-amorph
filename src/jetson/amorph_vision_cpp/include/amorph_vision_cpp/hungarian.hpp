// 匈牙利算法（最小代价二分匹配），用于 SORT 的检测-轨迹关联。
#pragma once

#include <vector>

namespace amorph::vision {

/// 求解矩形代价矩阵的最小代价完全匹配。
///
/// 要求所有元素有限；调用方需先用 gating 提取可行子矩阵（与 Python 版一致）。
/// @return assignment[row] = col；行数大于列数时内部转置处理。
std::vector<int> SolveAssignment(const std::vector<std::vector<double>>& cost);

}  // namespace amorph::vision
