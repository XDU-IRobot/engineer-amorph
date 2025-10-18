
#pragma once

#include <format>

namespace logging {

template <typename... Args>
inline void debug(const std::string &fmt, Args &&...args) {
  const auto str = std::format(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void info(const std::string &fmt, Args &&...args) {
  const auto str = std::format(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(const std::string &fmt, Args &&...args) {
  const auto str = std::format(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(const std::string &fmt, Args &&...args) {
  const auto str = std::format(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[noreturn]] inline void fatal(const std::string &fmt, Args &&...args) {
  const auto str = std::format(fmt, std::forward<Args>(args)...);
  while (true) {
  }
}

}  // namespace logging