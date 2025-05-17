
#ifndef KEYPRESS_HANDLER_HPP
#define KEYPRESS_HANDLER_HPP

#include <atomic>
#include <stdexcept>
#include <string>
#include <functional>
#include <unordered_map>
#include <utility>

namespace fluorite {

class Hotkey {
 public:
  explicit Hotkey(std::function<bool()> pred);
  Hotkey() = default;
  ~Hotkey() = default;

  inline Hotkey(const Hotkey &other) {
    have_pressed_ = other.have_pressed_.load();
    condition_ = other.condition_;
    pred_ = other.pred_;
    onhold_cb_ = other.onhold_cb_;
    onidle_cb_ = other.onidle_cb_;
    onpress_cb_ = other.onpress_cb_;
    onrelease_cb_ = other.onrelease_cb_;
  }
  inline Hotkey &operator=(const Hotkey &other) {
    if (this != &other) {
      have_pressed_ = other.have_pressed_.load();
      condition_ = other.condition_;
      pred_ = other.pred_;
      onhold_cb_ = other.onhold_cb_;
      onidle_cb_ = other.onidle_cb_;
      onpress_cb_ = other.onpress_cb_;
      onrelease_cb_ = other.onrelease_cb_;
    }
    return *this;
  }

  void Poll();
  void Poll(bool new_condition);
  Hotkey &OnPress(std::function<void()> cb);
  Hotkey &OnRelease(std::function<void()> cb);
  Hotkey &OnHold(std::function<void()> cb);
  Hotkey &OnIdle(std::function<void()> cb);
  bool have_pressed(bool reset = true);

 private:
  std::atomic<bool> have_pressed_{false};
  bool condition_{false};
  std::function<bool()> pred_{[]() { return false; }};
  std::function<void()> onhold_cb_{[]() {}};
  std::function<void()> onidle_cb_{[]() {}};
  std::function<void()> onpress_cb_{[]() {}};
  std::function<void()> onrelease_cb_{[]() {}};
};

class KeypressHandler {
 public:
  void Poll() {
    for (auto &key : hotkey_list_) {
      key->Poll();
    }
  }
  KeypressHandler &operator<<(Hotkey &key) {
    hotkey_list_.push_back(&key);
    return *this;
  }
  void ResetHavePressed() {
    for (auto &key : hotkey_list_) {
      key->have_pressed(true);
    }
  }

 private:
  std::vector<Hotkey *> hotkey_list_{};  // fuck raw ptr, will be refactored when i have the bandwidth
};

}  // namespace fluorite

#endif  // KEYPRESS_HANDLER_HPP