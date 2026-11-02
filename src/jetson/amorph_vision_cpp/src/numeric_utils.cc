#include "amorph_vision_cpp/numeric_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace amorph::vision {

double Median(std::vector<float> values) {
  if (values.empty()) {
    return 0.0;
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  const double upper = values[mid];
  if (values.size() % 2 != 0) {
    return upper;
  }
  // 偶数个：需要前半部分的最大值
  const double lower = *std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
  return 0.5 * (lower + upper);
}

double Percentile(std::vector<double> values, double percent) {
  if (values.empty()) {
    return 0.0;
  }
  if (values.size() == 1) {
    return values.front();
  }
  std::sort(values.begin(), values.end());

  const double rank = std::clamp(percent, 0.0, 100.0) / 100.0 * static_cast<double>(values.size() - 1);
  const auto low = static_cast<std::size_t>(std::floor(rank));
  const auto high = static_cast<std::size_t>(std::ceil(rank));
  if (low == high) {
    return values[low];
  }
  const double frac = rank - static_cast<double>(low);
  return values[low] + (values[high] - values[low]) * frac;
}

double HuberWeight(double residual, double k) {
  const double abs_r = std::abs(residual);
  if (abs_r <= k) {
    return 1.0;
  }
  return k / abs_r;
}

std::vector<std::size_t> LinspaceIndices(std::size_t n, std::size_t m) {
  std::vector<std::size_t> indices;
  if (n == 0 || m == 0) {
    return indices;
  }
  if (m >= n) {
    indices.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      indices[i] = i;
    }
    return indices;
  }
  indices.reserve(m);
  if (m == 1) {
    indices.push_back(0);
    return indices;
  }
  const double step = static_cast<double>(n - 1) / static_cast<double>(m - 1);
  for (std::size_t i = 0; i < m; ++i) {
    // np.linspace 生成浮点序列后 astype(int64) 向零截断（正数即 floor）
    auto idx = static_cast<std::size_t>(std::floor(static_cast<double>(i) * step));
    indices.push_back(std::min(idx, n - 1));
  }
  return indices;
}

}  // namespace amorph::vision
