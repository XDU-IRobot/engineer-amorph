
#ifndef KEYPRESS_HANDLER_HPP
#define KEYPRESS_HANDLER_HPP

#include <atomic>
#include <stdexcept>
#include <string>
#include <functional>
#include <unordered_map>

#include <utility>

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
  std::function<void()> onpress_cb_{[]() {}};
  std::function<void()> onrelease_cb_{[]() {}};
};

#endif  // KEYPRESS_HANDLER_HPP