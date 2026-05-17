
#pragma once

#include <rclcpp/rclcpp.hpp>

namespace amorph {

namespace text {
constexpr inline std::string Bold(const std::string& str) { return "\033[1m" + str + "\033[0m"; }
//
constexpr inline std::string Green(const std::string& str) { return "\033[32m" + str + "\033[0m"; }
constexpr inline std::string Red(const std::string& str) { return "\033[31m" + str + "\033[0m"; }
constexpr inline std::string Yellow(const std::string& str) { return "\033[33m" + str + "\033[0m"; }
//
constexpr inline std::string GreenBG(const std::string& str) { return "\033[42m" + str + "\033[0m"; }
constexpr inline std::string RedBG(const std::string& str) { return "\033[41m" + str + "\033[0m"; }
constexpr inline std::string YellowBG(const std::string& str) { return "\033[43m" + str + "\033[0m"; }

inline namespace literals {
constexpr inline std::string operator"" _bold(const char* str, std::size_t) { return Bold(str); }
constexpr inline std::string operator"" _green(const char* str, std::size_t) { return Green(str); }
constexpr inline std::string operator"" _red(const char* str, std::size_t) { return Red(str); }
constexpr inline std::string operator"" _yellow(const char* str, std::size_t) { return Yellow(str); }
constexpr inline std::string operator"" _greenbg(const char* str, std::size_t) { return GreenBG(str); }
constexpr inline std::string operator"" _redbg(const char* str, std::size_t) { return RedBG(str); }
constexpr inline std::string operator"" _yellowbg(const char* str, std::size_t) { return YellowBG(str); }
}  // namespace literals
}  // namespace text

struct L {
  static rclcpp::Logger& Get();

  template <typename... Args>
  static void D(const char* format, const Args&... args) {
    RCLCPP_DEBUG(Get(), format, args...);
  }
  template <typename... Args>
  static void D(const std::string& format, const Args&... args) {
    RCLCPP_DEBUG(Get(), format.c_str(), args...);
  }

  template <typename... Args>
  static void I(const char* format, const Args&... args) {
    RCLCPP_INFO(Get(), format, args...);
  }
  template <typename... Args>
  static void I(const std::string& format, const Args&... args) {
    RCLCPP_INFO(Get(), format.c_str(), args...);
  }

  template <typename... Args>
  static void W(const char* format, const Args&... args) {
    RCLCPP_WARN(Get(), format, args...);
  }
  template <typename... Args>
  static void W(const std::string& format, const Args&... args) {
    RCLCPP_WARN(Get(), format.c_str(), args...);
  }

  template <typename... Args>
  static void E(const char* format, const Args&... args) {
    RCLCPP_ERROR(Get(), format, args...);
  }
  template <typename... Args>
  static void E(const std::string& format, const Args&... args) {
    RCLCPP_ERROR(Get(), format.c_str(), args...);
  }

  template <typename... Args>
  static void F(const char* format, const Args&... args) {
    RCLCPP_FATAL(Get(), format, args...);
  }
  template <typename... Args>
  static void F(const std::string& format, const Args&... args) {
    RCLCPP_FATAL(Get(), format.c_str(), args...);
  }
};

}  // namespace amorph