
#ifndef KEYPRESS_HANDLER_HPP
#define KEYPRESS_HANDLER_HPP

#include <atomic>
#include <stdexcept>
#include <string>
#include <functional>
#include <unordered_map>

#include <edge_trigger.hpp>
#include <utility>

class Hotkey {
 public:
  explicit inline Hotkey(std::function<bool()> pred) : pred_(std::move(pred)) {
    edge_trigger_.on_rising([this] { have_pressed_.store(true); });
  }
  Hotkey() = default;
  ~Hotkey() = default;

  inline void Poll() {
    if (pred_) {
      condition_ = pred_();
    }
  }

  inline void Poll(bool new_condition) { condition_ = new_condition; }

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

  inline bool have_pressed(bool reset = true) { return reset ? have_pressed_.exchange(false) : have_pressed_.load(); }

 private:
  std::atomic<bool> have_pressed_{false};
  bool condition_{false};
  std::function<bool()> pred_{[]() { return false; }};
  std::function<void()> onhold_cb_{[]() {}};
  std::function<void()> onidle_cb_{[]() {}};
  lunarifish::edge_trigger<bool> edge_trigger_{condition_};
};

// class KeypressHandler {
//  public:
//   void Poll() {
//     for (auto &key : hotkey_list_) {
//       key.Poll();
//     }
//   }
//   KeypressHandler &operator<<(Hotkey key) {
//     hotkey_list_.push_back(std::move(key));
//     return *this;
//   }

//  private:
//   std::vector<Hotkey> hotkey_list_{};
// };

#endif  // KEYPRESS_HANDLER_HPP