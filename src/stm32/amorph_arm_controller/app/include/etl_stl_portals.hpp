
#pragma once

#include <vector>

#include <etl/vector.h>

// etl::vector转换为std::vector
template <typename T, size_t N>
std::vector<T> etl_vector_to_std_vector(const etl::vector<T, N>& etl_vec) {
  return std::vector<T>(etl_vec.begin(), etl_vec.end());
}