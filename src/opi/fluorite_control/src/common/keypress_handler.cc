
#include <iostream>

#include "common/keypress_handler.hpp"

namespace fluorite {

Hotkey::Hotkey(std::function<bool()> pred) : pred_(std::move(pred)) {
  // edge_trigger_.on_rising([]{std::cout << "rising!" << std::endl;}).on_rising([this] { have_pressed_.store(true); });
}

void Hotkey::Poll() {
  if (pred_) {
    condition_ = pred_();
    // edge_trigger_.update();
    if (condition_ && onhold_cb_) {
      onhold_cb_();
    }
    if (!condition_ && onidle_cb_) {
      onidle_cb_();
    }
  }
}

void Hotkey::Poll(bool new_condition) {
  if ((condition_ != new_condition) && new_condition) {
    have_pressed_.store(true);
  }
  condition_ = new_condition;
  // edge_trigger_.update();
  if (condition_ && onhold_cb_) {
    onhold_cb_();
  }
  if (!condition_ && onidle_cb_) {
    onidle_cb_();
  }
}

Hotkey &Hotkey::OnPress(std::function<void()> cb) {
  edge_trigger_.on_rising(cb);
  return *this;
}

Hotkey &Hotkey::OnRelease(std::function<void()> cb) {
  edge_trigger_.on_falling(cb);
  return *this;
}

Hotkey &Hotkey::OnHold(std::function<void()> cb) {
  onhold_cb_ = cb;
  return *this;
}

Hotkey &Hotkey::OnIdle(std::function<void()> cb) {
  onidle_cb_ = cb;
  return *this;
}

bool Hotkey::have_pressed(bool reset) { return reset ? have_pressed_.exchange(false) : have_pressed_.load(); }

}  // namespace fluorite