// 与 numpy 语义对齐的统计工具。
#pragma once

#include <vector>

namespace amorph::vision {

/// 中位数，与 np.median 一致：偶数个元素时取中间两个的平均值。
double Median(std::vector<float> values);

/// 百分位数（0~100），与 np.percentile 默认的线性插值一致。
double Percentile(std::vector<double> values, double percent);

/// Huber 鲁棒权重：|r| <= k 时为 1，否则为 k / |r|。
double HuberWeight(double residual, double k);

/// 与 np.linspace(0, n - 1, m).astype(int64) 一致的等间隔下标。
std::vector<std::size_t> LinspaceIndices(std::size_t n, std::size_t m);

}  // namespace amorph::vision
