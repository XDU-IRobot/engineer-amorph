
#ifndef ENCODER_ARRAY_HPP
#define ENCODER_ARRAY_HPP

#include <Arduino.h>

#include <array>

#include "AS5600.h"
#include "TCA9548.h"

class EncoderArray {
 public:
  explicit EncoderArray(HardwareSerial &logger) : logger_(&logger) {}

  void Begin() {
    Wire.begin();

    if (multiplexer_.begin() == false) {
      logger_->println("Could not connect to TCA9548 multiplexer.");
    }

    RescanSensors();
    encoder_.begin();
  }

  AS5600 *operator[](int channel) {
    assert(channel >= 0 && channel < 6);
    if (!encoders_ok_[channel]) {
      return nullptr;
    }
    multiplexer_.selectChannel(channel);
    return &encoder_;
  }

  void RescanSensors() {
    for (int chan = 0; chan < 6; chan++) {
      multiplexer_.selectChannel(chan);
      bool b = multiplexer_.isConnected(kAS5600Addr);
      encoders_ok_[chan] = b;
    }
  }

  [[nodiscard]] bool ok() const {
    return std::all_of(encoders_ok_.begin(), encoders_ok_.end(), [](const bool b) { return b; });
  }

 private:
  static constexpr uint8_t kAS5600Addr = 0x36;

  HardwareSerial *logger_;

  TCA9548 multiplexer_{0x70};
  AS5600 encoder_{};
  std::array<bool, 6> encoders_ok_{};
};

#endif  // ENCODER_ARRAY_HPP
