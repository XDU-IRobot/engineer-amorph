
#ifndef KEYPRESS_HANDLER_HPP
#define KEYPRESS_HANDLER_HPP

#include <atomic>
#include <stdexcept>
#include <string>
#include <functional>
#include <unordered_map>
#include <utility>

// class Hotkey {
//   friend class KeypressHandler;

//  public:
//   explicit Hotkey(std::function<bool()> pred);
//   Hotkey() = default;
//   ~Hotkey() = default;

//   inline Hotkey(const Hotkey &other) {
//     have_pressed_ = other.have_pressed_.load();
//     condition_ = other.condition_;
//     pred_ = other.pred_;
//     onhold_cb_ = other.onhold_cb_;
//     onidle_cb_ = other.onidle_cb_;
//     onpress_cb_ = other.onpress_cb_;
//     onrelease_cb_ = other.onrelease_cb_;
//   }
//   inline Hotkey &operator=(const Hotkey &other) {
//     if (this != &other) {
//       have_pressed_ = other.have_pressed_.load();
//       condition_ = other.condition_;
//       pred_ = other.pred_;
//       onhold_cb_ = other.onhold_cb_;
//       onidle_cb_ = other.onidle_cb_;
//       onpress_cb_ = other.onpress_cb_;
//       onrelease_cb_ = other.onrelease_cb_;
//     }
//     return *this;
//   }

//   void Poll();
//   void Poll(bool new_condition);
//   Hotkey &OnPress(std::function<void()> cb);
//   Hotkey &OnRelease(std::function<void()> cb);
//   Hotkey &OnHold(std::function<void()> cb);
//   Hotkey &OnIdle(std::function<void()> cb);
//   bool have_pressed(bool reset = true);

//  private:
//   std::atomic<bool> enable_{true};
//   std::atomic<bool> have_pressed_{false};
//   bool condition_{false};
//   std::function<bool()> pred_{[]() { return false; }};
//   std::function<void()> onhold_cb_{[]() {}};
//   std::function<void()> onidle_cb_{[]() {}};
//   std::function<void()> onpress_cb_{[]() {}};
//   std::function<void()> onrelease_cb_{[]() {}};
// };

inline constexpr size_t HammingWeight(uint16_t val) {
  size_t count = 0;
  while (val) {
    count += val & 1;
    val >>= 1;
  }
  return count;
}

class Hotkey {
  friend class KeypressHandler;

 public:
  Hotkey() : keycode_mask_{0u}, is_combo_{false} {}
  explicit Hotkey(uint16_t keycode_mask) : keycode_mask_{keycode_mask}, is_combo_{HammingWeight(keycode_mask_) > 1} {}

  Hotkey(const Hotkey &other) : keycode_mask_{other.keycode_mask_}, is_combo_{other.is_combo_} {
    have_pressed_.store(other.have_pressed_.load());
    condition_.store(other.condition_.load());
  }
  Hotkey &operator=(const Hotkey &other) {
    if (this != &other) {
      keycode_mask_ = other.keycode_mask_;
      is_combo_ = other.is_combo_;
      have_pressed_.store(other.have_pressed_.load());
      condition_.store(other.condition_.load());
    }
    return *this;
  }

  void Poll(uint16_t keycode) {
    const bool previous_condition = condition_.load();
    if (is_combo_) {
      // 对于组合键，必须在所有按键都松开时才算松开，所有按键都按下时才算按下
      if (previous_condition) {
        condition_.store((keycode & keycode_mask_) != 0u);
      } else {
        condition_.store((keycode & keycode_mask_) == keycode_mask_);
      }
    } else {
      condition_.store((keycode & keycode_mask_) != 0u);
    }

    if (condition_ && !previous_condition) {
      have_pressed_.store(true);
      if (onpress_cb_) {
        onpress_cb_();
      }
    } else if (!condition_ && previous_condition) {
      if (onrelease_cb_) {
        onrelease_cb_();
      }
    }
  }

  Hotkey &OnPress(std::function<void()> cb) {
    onpress_cb_ = std::move(cb);
    return *this;
  }
  Hotkey &OnRelease(std::function<void()> cb) {
    onrelease_cb_ = std::move(cb);
    return *this;
  }

  uint16_t mask() const { return keycode_mask_; }
  bool condition() const { return condition_.load(); }
  bool have_pressed(bool reset = true) { return reset ? have_pressed_.exchange(false) : have_pressed_.load(); }

 private:
  uint16_t keycode_mask_;
  bool is_combo_;

  std::atomic<bool> have_pressed_{false};
  std::atomic<bool> condition_{false};
  std::function<void()> onpress_cb_{[]() {}};
  std::function<void()> onrelease_cb_{[]() {}};
};

class KeypressHandler {
 public:
  void Poll(uint16_t keycode) {
    combo_freeze_mask_.store(0u);
    for (auto &combo : combo_list_) {
      combo->Poll(keycode);
      if (combo->condition()) {
        // 组合键按下，屏蔽这个组合键的各个单键
        combo_freeze_mask_.store(combo->keycode_mask_);
        break;  // 一次只允许一个组合键按下
      }
    }
    for (auto &key : single_list_) {
      if ((key->keycode_mask_ & combo_freeze_mask_.load()) != 0u) {
        // 这个单键被组合键屏蔽了
        key->Poll(0u);
      } else {
        key->Poll(keycode);
      }
    }
  }
  KeypressHandler &operator<<(Hotkey &key) {
    if (key.is_combo_) {
      combo_list_.push_back(&key);
    } else {
      single_list_.push_back(&key);
    }
    return *this;
  }

 private:
  std::atomic<uint16_t> combo_freeze_mask_{0u};  // 有正在进行的组合键就屏蔽这个组合键的各个单键
  std::vector<Hotkey *> combo_list_{};           // fuck raw ptr, will be refactored when i have the bandwidth
  std::vector<Hotkey *> single_list_{};          // fuck raw ptr, will be refactored when i have the bandwidth
};

#endif  // KEYPRESS_HANDLER_HPP