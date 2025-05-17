
#include <iostream>

#include "common/keypress_handler.hpp"

namespace fluorite {

Hotkey::Hotkey(std::function<bool()> pred) : pred_(std::move(pred)) {}

void Hotkey::Poll() {
  if (pred_) {
    const bool new_condition = pred_();
    if (condition_ != new_condition) {
      if (new_condition) {
        have_pressed_.store(true);
        if (onpress_cb_) {
          onpress_cb_();
        }
      } else {
        if (onrelease_cb_) {
          onrelease_cb_();
        }
      }
    }
    condition_ = new_condition;
    if (condition_ && onhold_cb_) {
      onhold_cb_();
    }
    if (!condition_ && onidle_cb_) {
      onidle_cb_();
    }
  }
}

void Hotkey::Poll(bool new_condition) {
  if (condition_ != new_condition) {
    if (new_condition) {
      have_pressed_.store(true);
      if (onpress_cb_) {
        onpress_cb_();
      }
    } else {
      if (onrelease_cb_) {
        onrelease_cb_();
      }
    }
  }
  condition_ = new_condition;
  if (condition_ && onhold_cb_) {
    onhold_cb_();
  }
  if (!condition_ && onidle_cb_) {
    onidle_cb_();
  }
}

Hotkey &Hotkey::OnPress(std::function<void()> cb) {
  onpress_cb_ = std::move(cb);
  return *this;
}

Hotkey &Hotkey::OnRelease(std::function<void()> cb) {
  onrelease_cb_ = std::move(cb);
  return *this;
}

Hotkey &Hotkey::OnHold(std::function<void()> cb) {
  onhold_cb_ = std::move(cb);
  return *this;
}

Hotkey &Hotkey::OnIdle(std::function<void()> cb) {
  onidle_cb_ = std::move(cb);
  return *this;
}

bool Hotkey::have_pressed(bool reset) { return reset ? have_pressed_.exchange(false) : have_pressed_.load(); }
}  // namespace fluorite