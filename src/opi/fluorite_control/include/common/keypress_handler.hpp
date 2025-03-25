
#ifndef KEYPRESS_HANDLER_HPP
#define KEYPRESS_HANDLER_HPP

#include <stdexcept>
#include <string>
#include <functional>
#include <unordered_map>

#include <edge_trigger.hpp>

class Hotkey {
 public:
  Hotkey() = delete;
  explicit inline Hotkey(std::function<bool()> pred) : pred_(pred) {}
  ~Hotkey() = default;

  inline void Poll() { condition_ = pred_(); }

  inline Hotkey &OnPress(std::function<void()> cb) {
    edge_trigger_.on_rising(cb);
    return *this;
  }

  inline Hotkey &OnRelease(std::function<void()> cb) {
    edge_trigger_.on_falling(cb);
    return *this;
  }

  inline Hotkey &OnHold(std::function<void()> cb) {
    onhold_cb_ = cb;
    return *this;
  }

  inline Hotkey &OnIdle(std::function<void()> cb) {
    onidle_cb_ = cb;
    return *this;
  }

 private:
  bool condition_{false};
  std::function<bool()> pred_{[]() { return false; }};
  std::function<void()> onhold_cb_{[]() {}};
  std::function<void()> onidle_cb_{[]() {}};
  lunarifish::edge_trigger<bool> edge_trigger_{condition_};
};

class KeypressHandler {
  void Poll() {
    for (auto &key : hotkey_list_) {
      key.Poll();
    }
  }
  void operator<<(const Hotkey &key) { hotkey_list_.push_back(key); }

 private:
  std::vector<Hotkey> hotkey_list_{};
};

#endif  // KEYPRESS_HANDLER_HPP