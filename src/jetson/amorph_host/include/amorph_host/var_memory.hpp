#pragma once

#include <source_location>
#include <unordered_map>
#include <string_view>
#include <functional>
#include <cstdint>

/**
 * @brief 用于记录变量上一次值的工具类，基于调用位置（Source Location）区分不同变量
 * @note 需要 C++20 支持
 *
 * 用法示例:
 * if (msg->failsafe && !VarMemory::last(msg->failsafe)) {
 *   // 检测到 failsafe 上升沿
 * }
 */
class VarMemory {
  struct Key {
    const char* file;
    std::uint_least32_t line;
    std::uint_least32_t col;
  };

  struct KeyHash {
    std::size_t operator()(const Key& k) const {
      std::size_t h1 = std::hash<std::string_view>{}(k.file);
      std::size_t h2 = std::hash<std::uint_least32_t>{}(k.line);
      std::size_t h3 = std::hash<std::uint_least32_t>{}(k.col);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  struct KeyEq {
    bool operator()(const Key& lhs, const Key& rhs) const {
      return lhs.line == rhs.line && lhs.col == rhs.col && std::string_view(lhs.file) == std::string_view(rhs.file);
    }
  };

  template <typename T>
  static auto& GetStorage() {
    static std::unordered_map<Key, T, KeyHash, KeyEq> storage;
    return storage;
  }

 public:
  /**
   * @brief 获取变量的上一次值，并更新为当前值
   * @param current 当前值
   * @param loc 调用位置（自动获取，无需手动传入）
   * @return T 上一次的值（如果是第一次调用，返回默认构造的值，如 bool 为 false）
   */
  template <typename T>
  static T last(const T& current, std::source_location loc = std::source_location::current()) {
    auto& storage = GetStorage<T>();
    Key key{loc.file_name(), loc.line(), loc.column()};

    auto it = storage.find(key);
    if (it == storage.end()) {
      storage[key] = current;
      return T{};
    }
    T prev = it->second;
    it->second = current;
    return prev;
  }
};

