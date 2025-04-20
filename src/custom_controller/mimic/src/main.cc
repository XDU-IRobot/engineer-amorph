
#include <Arduino.h>
#include <HardwareSerial.h>

#include "led.hpp"
#include "packet_builder.hpp"
#include "encoder_array.hpp"

HardwareSerial debug_serial(1);
EncoderArray encoders(debug_serial);

[[noreturn]] void LedTask(void *arg) {
  RGBLed led;
  led.Begin();
  led.StandBy();
  while (true) {
    // if (encoders.ok()) {
    //   led.GreenBreath();
    // } else {
    //   led.RedBreath();
    // }
    led.Update();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

[[noreturn]] void MainTask(void *arg) {
  HardwareSerial rs232(2);
  PacketBuilder pb;
  rs232.begin(115200, SERIAL_8N1, 39, 38);
  encoders.Begin();

  struct __attribute__((__packed__)) {
    bool ok;
    float joint_positions[6];
  } tx_message{};

  while (true) {
    debug_serial.println("===================================");
    encoders.RescanSensors();
    for (int i = 0; i < 6; ++i) {
      auto encoder = encoders[i];
      if (encoder == nullptr) {
        debug_serial.println("sensor offline");
        continue;
      }
      tx_message.joint_positions[i] = encoder->readAngle() * AS5600_RAW_TO_RADIANS;
      debug_serial.println(tx_message.joint_positions[i]);
    }
    tx_message.ok = encoders.ok();
    const auto packet = pb.Build(reinterpret_cast<uint8_t *>(&tx_message), sizeof(tx_message));
    for (const auto c : *packet) {
      rs232.write(c);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup() {
  debug_serial.begin(921600, SERIAL_8N1, 44, 43);
  xTaskCreate(LedTask, "LedTask", 1024, nullptr, configMAX_PRIORITIES - 2, nullptr);
  xTaskCreate(MainTask, "MainTask", 2048, nullptr, configMAX_PRIORITIES - 1, nullptr);
}

void loop() { vTaskDelay(portMAX_DELAY); }
