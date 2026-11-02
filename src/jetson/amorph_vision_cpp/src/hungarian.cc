#include "amorph_vision_cpp/hungarian.hpp"

#include <algorithm>
#include <limits>

namespace amorph::vision {
namespace {

/// 经典 O(n^3) 匈牙利算法（势函数 + 最短增广路），要求 rows <= cols。
std::vector<int> SolveSquare(const std::vector<std::vector<double>>& cost, std::size_t rows, std::size_t cols) {
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<double> u(rows + 1, 0.0);
  std::vector<double> v(cols + 1, 0.0);
  std::vector<std::size_t> p(cols + 1, 0);  // p[j] = 与列 j 匹配的行
  std::vector<std::size_t> way(cols + 1, 0);

  for (std::size_t i = 1; i <= rows; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::vector<double> minv(cols + 1, kInf);
    std::vector<bool> used(cols + 1, false);
    do {
      used[j0] = true;
      const std::size_t i0 = p[j0];
      double delta = kInf;
      std::size_t j1 = 0;
      for (std::size_t j = 1; j <= cols; ++j) {
        if (used[j]) {
          continue;
        }
        const double current = cost[i0 - 1][j - 1] - u[i0] - v[j];
        if (current < minv[j]) {
          minv[j] = current;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (std::size_t j = 0; j <= cols; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(rows, -1);
  for (std::size_t j = 1; j <= cols; ++j) {
    if (p[j] == 0 || p[j] > rows) {
      continue;
    }
    assignment[p[j] - 1] = static_cast<int>(j - 1);
  }
  return assignment;
}

}  // namespace

std::vector<int> SolveAssignment(const std::vector<std::vector<double>>& cost) {
  if (cost.empty() || cost.front().empty()) {
    return {};
  }
  const std::size_t rows = cost.size();
  const std::size_t cols = cost.front().size();

  if (rows <= cols) {
    return SolveSquare(cost, rows, cols);
  }

  // 行多于列时转置，结果再映射回原下标
  std::vector<std::vector<double>> transposed(cols, std::vector<double>(rows, 0.0));
  for (std::size_t i = 0; i < rows; ++i) {
    for (std::size_t j = 0; j < cols; ++j) {
      transposed[j][i] = cost[i][j];
    }
  }
  const std::vector<int> transposed_assignment = SolveSquare(transposed, cols, rows);

  std::vector<int> assignment(rows, -1);
  for (std::size_t j = 0; j < transposed_assignment.size(); ++j) {
    const int row = transposed_assignment[j];
    if (row >= 0) {
      assignment[static_cast<std::size_t>(row)] = static_cast<int>(j);
    }
  }
  return assignment;
}

}  // namespace amorph::vision
