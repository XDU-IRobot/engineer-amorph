
#pragma once

#include <vector>
#include <algorithm>
// #include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/LU>

enum class SplineType {
  kLinear,       ///< 线性样条
  kCubic,        ///< 三次样条（自然边界条件）
  kCubicHermite  ///< 三次Hermite样条
};

/**
 * @brief   一维三次样条曲线插值器
 */
template <typename T = float>
class Spline1D {
 private:
  std::vector<T> x_data_;
  std::vector<T> y_data_;
  std::vector<T> coefficients_;
  SplineType spline_type_;
  bool initialized_;

  [[noreturn]] void Fatal() const {
    while (true) {
    }
  }

  // 三对角矩阵求解器 (Thomas algorithm)
  void SolveTridiagonal(const std::vector<T>& a, const std::vector<T>& b, const std::vector<T>& c,
                        const std::vector<T>& d, std::vector<T>& x) {
    size_t n = d.size();
    std::vector<T> c_prime(n);
    std::vector<T> d_prime(n);

    c_prime[0] = c[0] / b[0];
    d_prime[0] = d[0] / b[0];

    for (size_t i = 1; i < n; ++i) {
      T denom = b[i] - a[i] * c_prime[i - 1];
      c_prime[i] = c[i] / denom;
      d_prime[i] = (d[i] - a[i] * d_prime[i - 1]) / denom;
    }

    x[n - 1] = d_prime[n - 1];
    for (int i = n - 2; i >= 0; --i) {
      x[i] = d_prime[i] - c_prime[i] * x[i + 1];
    }
  }

  void ComputeLinearSpline() {
    coefficients_.clear();
    coefficients_.reserve((x_data_.size() - 1) * 2);

    for (size_t i = 0; i < x_data_.size() - 1; ++i) {
      T slope = (y_data_[i + 1] - y_data_[i]) / (x_data_[i + 1] - x_data_[i]);
      coefficients_.push_back(y_data_[i]);  // 常数项
      coefficients_.push_back(slope);       // 一次项系数
    }
  }

  void ComputeCubicSpline() {
    size_t n = x_data_.size();
    if (n < 3) {
      ComputeLinearSpline();
      return;
    }

    // 设置自然边界条件 (二阶导数为0)
    std::vector<T> h(n - 1), alpha(n - 1);
    std::vector<T> l(n), mu(n), z(n);
    std::vector<T> c(n);

    // 计算步长
    for (size_t i = 0; i < n - 1; ++i) {
      h[i] = x_data_[i + 1] - x_data_[i];
    }

    // 计算alpha
    for (size_t i = 1; i < n - 1; ++i) {
      alpha[i] = 3.0 / h[i] * (y_data_[i + 1] - y_data_[i]) - 3.0 / h[i - 1] * (y_data_[i] - y_data_[i - 1]);
    }

    // 设置边界条件
    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;
    l[n - 1] = 1.0;
    z[n - 1] = 0.0;

    // 解三对角矩阵
    for (size_t i = 1; i < n - 1; ++i) {
      l[i] = 2.0 * (x_data_[i + 1] - x_data_[i - 1]) - h[i - 1] * mu[i - 1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    // 计算c系数 (二阶导数)
    for (int i = n - 2; i >= 0; --i) {
      c[i] = z[i] - mu[i] * c[i + 1];
    }

    // 计算所有系数
    coefficients_.clear();
    coefficients_.reserve((n - 1) * 4);

    for (size_t i = 0; i < n - 1; ++i) {
      T a = y_data_[i];
      T b = (y_data_[i + 1] - y_data_[i]) / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0;
      T d = (c[i + 1] - c[i]) / (3.0 * h[i]);

      coefficients_.push_back(a);
      coefficients_.push_back(b);
      coefficients_.push_back(c[i]);
      coefficients_.push_back(d);
    }
  }

  void ComputeCubicHermiteSpline() {
    size_t n = x_data_.size();
    coefficients_.clear();
    coefficients_.reserve((n - 1) * 4);

    for (size_t i = 0; i < n - 1; ++i) {
      T h = x_data_[i + 1] - x_data_[i];
      T y0 = y_data_[i];
      T y1 = y_data_[i + 1];

      // 使用中心差分计算导数
      T m0, m1;
      if (i == 0) {
        m0 = (y_data_[i + 1] - y_data_[i]) / h;
      } else {
        m0 = (y_data_[i + 1] - y_data_[i - 1]) / (x_data_[i + 1] - x_data_[i - 1]);
      }

      if (i == n - 2) {
        m1 = (y_data_[i + 1] - y_data_[i]) / h;
      } else {
        m1 = (y_data_[i + 2] - y_data_[i]) / (x_data_[i + 2] - x_data_[i]);
      }

      // Hermite 基函数系数
      T a = y0;
      T b = m0;
      T c = 3.0 * (y1 - y0) / (h * h) - (2.0 * m0 + m1) / h;
      T d = 2.0 * (y0 - y1) / (h * h * h) + (m0 + m1) / (h * h);

      coefficients_.push_back(a);
      coefficients_.push_back(b);
      coefficients_.push_back(c);
      coefficients_.push_back(d);
    }
  }

  size_t FindSegment(T x) const {
    if (x <= x_data_.front()) return 0;
    if (x >= x_data_.back()) return x_data_.size() - 2;

    auto it = std::upper_bound(x_data_.begin(), x_data_.end(), x);
    return std::distance(x_data_.begin(), it) - 1;
  }

 public:
  Spline1D(SplineType type = SplineType::kCubic) : spline_type_(type), initialized_(false) {}

  template <typename InputIt1, typename InputIt2>
  Spline1D(InputIt1 x_begin, InputIt1 x_end, InputIt2 y_begin, InputIt2 y_end, SplineType type = SplineType::kCubic)
      : spline_type_(type), initialized_(false) {
    SetData(x_begin, x_end, y_begin, y_end);
  }

  Spline1D(const std::vector<T>& x, const std::vector<T>& y, SplineType type = SplineType::kCubic)
      : spline_type_(type), initialized_(false) {
    SetData(x, y);
  }

  template <typename InputIt1, typename InputIt2>
  void SetData(InputIt1 x_begin, InputIt1 x_end, InputIt2 y_begin, InputIt2 y_end) {
    x_data_.assign(x_begin, x_end);
    y_data_.assign(y_begin, y_end);

    if (x_data_.size() != y_data_.size()) {
      Fatal();
      // throw std::invalid_argument("x and y data must have same size");
    }
    if (x_data_.size() < 2) {
      Fatal();
      // throw std::invalid_argument("At least 2 data points required");
    }

    // 检查x是否严格递增
    for (size_t i = 1; i < x_data_.size(); ++i) {
      if (x_data_[i] <= x_data_[i - 1]) {
        Fatal();
        // throw std::invalid_argument("x data must be strictly increasing");
      }
    }

    ComputeCoefficients();
    initialized_ = true;
  }

  void SetData(const std::vector<T>& x, const std::vector<T>& y) { SetData(x.begin(), x.end(), y.begin(), y.end()); }

  void SetType(SplineType type) {
    if (type != spline_type_) {
      spline_type_ = type;
      if (initialized_) {
        ComputeCoefficients();
      }
    }
  }

  void ComputeCoefficients() {
    switch (spline_type_) {
      case SplineType::kLinear:
        ComputeLinearSpline();
        break;
      case SplineType::kCubic:
        ComputeCubicSpline();
        break;
      case SplineType::kCubicHermite:
        ComputeCubicHermiteSpline();
        break;
    }
  }

  T operator()(T x) const {
    if (!initialized_) {
      Fatal();
      // throw std::runtime_error("Spline not initialized with data");
    }

    size_t seg = FindSegment(x);
    T dx = x - x_data_[seg];

    switch (spline_type_) {
      case SplineType::kLinear: {
        T a = coefficients_[seg * 2];
        T b = coefficients_[seg * 2 + 1];
        return a + b * dx;
      }

      case SplineType::kCubic:
      case SplineType::kCubicHermite: {
        T a = coefficients_[seg * 4];
        T b = coefficients_[seg * 4 + 1];
        T c = coefficients_[seg * 4 + 2];
        T d = coefficients_[seg * 4 + 3];
        return a + b * dx + c * dx * dx + d * dx * dx * dx;
      }
    }
    return T(0);
  }

  T Derivative(T x, int order = 1) const {
    if (!initialized_) {
      Fatal();
      /// throw std::runtime_error("Spline not initialized with data");
    }
    if (order < 1 || order > 3) {
      Fatal();
      // throw std::invalid_argument("Derivative order must be 1, 2, or 3");
    }

    size_t seg = FindSegment(x);
    T dx = x - x_data_[seg];

    switch (spline_type_) {
      case SplineType::kLinear: {
        if (order == 1) {
          return coefficients_[seg * 2 + 1];
        } else {
          return T(0);
        }
      }

      case SplineType::kCubic:
      case SplineType::kCubicHermite: {
        T a = coefficients_[seg * 4];
        T b = coefficients_[seg * 4 + 1];
        T c = coefficients_[seg * 4 + 2];
        T d = coefficients_[seg * 4 + 3];

        switch (order) {
          case 1:
            return b + 2.0 * c * dx + 3.0 * d * dx * dx;
          case 2:
            return 2.0 * c + 6.0 * d * dx;
          case 3:
            return 6.0 * d;
        }
      }
    }
    return T(0);
  }

  // 获取原始数据
  const std::vector<T>& x_data() const { return x_data_; }
  const std::vector<T>& y_data() const { return y_data_; }
  SplineType spline_type() const { return spline_type_; }
  bool initialized() const { return initialized_; }

  // 在多个点上同时计算插值
  Eigen::VectorXd Evaluate(const Eigen::VectorXd& x_vec) const {
    if (!initialized_) {
      Fatal();
      // throw std::runtime_error("Spline not initialized with data");
    }

    Eigen::VectorXd result(x_vec.size());
    for (Eigen::Index i = 0; i < x_vec.size(); ++i) {
      result(i) = operator()(x_vec(i));
    }
    return result;
  }
};
