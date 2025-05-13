
#ifndef KEYPRESS_HANDLER_HPP
#define KEYPRESS_HANDLER_HPP

#include <atomic>
#include <stdexcept>
#include <string>
#include <functional>
#include <unordered_map>

#include <edge_trigger.hpp>
#include <utility>

namespace fluorite {

class Hotkey {
 public:
  explicit inline Hotkey(std::function<bool()> pred);
  Hotkey() = default;
  ~Hotkey() = default;

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

}  // namespace fluorite

#endif  // KEYPRESS_HANDLER_HPP