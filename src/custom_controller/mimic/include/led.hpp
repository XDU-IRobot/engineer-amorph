
#ifndef LED_HPP
#define LED_HPP

#include <Arduino.h>

#include "Tween.h"

#include <algorithm>
#include <tuple>

extern HardwareSerial debug_serial;

struct RGBTween {
  using SharedPtr = std::shared_ptr<RGBTween>;
  Tween::Timeline r;
  Tween::Timeline g;
  Tween::Timeline b;
  void Start() {
    r.start();
    g.start();
    b.start();
  }
  void Update() {
    r.update();
    g.update();
    b.update();
  }
};

class RGBLed {
 public:
  ~RGBLed() {
    ledcDetachPin(LED_R);
    ledcDetachPin(LED_G);
    ledcDetachPin(LED_B);
  }

  void Begin() {
    ledcSetup(0, 1000, 8);
    ledcSetup(1, 1000, 8);
    ledcSetup(2, 1000, 8);
    ledcAttachPin(LED_R, 0);
    ledcAttachPin(LED_G, 1);
    ledcAttachPin(LED_B, 2);
    Off();
  }
  void Off() {
    if (current_mode_ == "Off") {
      return;
    }
    current_mode_ = "Off";
    current_tween_ = std::make_shared<RGBTween>();
    current_tween_->r.add(gv_).init(0.0f);
    current_tween_->g.add(gv_).init(0.0f);
    current_tween_->b.add(bv_).init(0.0f);
  }

  void RedBreath() {
    if (current_mode_ == "RedBreath") {
      return;
    }
    current_mode_ = "RedBreath";
    current_tween_ = std::make_shared<RGBTween>();
    current_tween_->r.mode(Tween::Mode::REPEAT_TL);
    current_tween_->g.add(gv_).init(0.0f);
    current_tween_->b.add(bv_).init(0.0f);
    current_tween_->r.add(rv_)
        .init(0.0f)       //
        .then(1.0f, 50)   //
        .hold(450)        //
        .then(0.0f, 300)  //
        .hold(200);
    current_tween_->Start();
  }

  void GreenBreath() {
    if (current_mode_ == "GreenBreath") {
      return;
    }
    current_mode_ = "GreenBreath";
    current_tween_ = std::make_shared<RGBTween>();
    current_tween_->g.mode(Tween::Mode::REPEAT_TL);
    current_tween_->r.add(rv_).init(0.0f);
    current_tween_->b.add(bv_).init(0.0f);
    current_tween_->g.add(gv_)
        .init(0.0f)       //
        .then(1.0f, 200)  //
        .hold(1300)       //
        .then(0.0f, 800)  //
        .hold(700);
    current_tween_->Start();
  }

  void StandBy() {
    if (current_mode_ == "StandBy") {
      return;
    }
    current_mode_ = "StandBy";
    current_tween_ = std::make_shared<RGBTween>();
    current_tween_->b.mode(Tween::Mode::REPEAT_TL);
    current_tween_->r.add(rv_).init(0.0f);
    current_tween_->g.add(gv_).init(0.0f);
    current_tween_->b.add(bv_)
        .init(0.0f)       //
        .then(0.5f, 200)  //
        .hold(1300)       //
        .then(0.0f, 800)  //
        .hold(700);
    current_tween_->Start();
  }

  void Update() {
    current_tween_->Update();
    ledcWrite(0, 255 - std::clamp(rv_, 0.0f, 1.0f) * 255);
    ledcWrite(1, 255 - std::clamp(gv_, 0.0f, 1.0f) * 255);
    ledcWrite(2, 255 - std::clamp(bv_, 0.0f, 1.0f) * 255);
  }

  // private:
  static constexpr int LED_R = 42;
  static constexpr int LED_G = 41;
  static constexpr int LED_B = 40;

  std::string current_mode_{};
  RGBTween::SharedPtr current_tween_{nullptr};
  float rv_{}, gv_{}, bv_{};
};

#endif  // LED_HPP
