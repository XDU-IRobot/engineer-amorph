
#include <Arduino.h>
#include <HardwareSerial.h>

#include "led.hpp"
#include "packet_builder.hpp"
#include "encoder_array.hpp"
#include "settings.hpp"

HardwareSerial debug_serial(1);
EncoderArray encoders(debug_serial);

TaskHandle_t led_task_handle, main_task_handle;

[[noreturn]] void LedTask(void *arg) {
  RGBLed led;
  led.Begin();
  ulTaskNotifyTake(pdTRUE, 2000 / portTICK_PERIOD_MS);
  while (true) {
    if (encoders.ok()) {
      led.GreenBreath();
    } else {
      led.RedBreath();
    }
    led.Update();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void JointPosPostProcess(float jp_array[6]) {
  for (size_t i = 0; i < 6; i++) {
    if (settings::kEncoderReverse[i]) {
      jp_array[i] = M_PI * 2 - jp_array[i];
    }
    jp_array[i] -= settings::kEncoderOffsets[i];
    jp_array[i] = rm::LoopConstrain(jp_array[i], -M_PI, M_PI);
    jp_array[i] = std::clamp(jp_array[i], settings::kJointLimits[i].first, settings::kJointLimits[i].second);
  }
};

[[noreturn]] void MainTask(void *arg) {
  HardwareSerial rs232(2);
  PacketBuilder pb;
  rs232.begin(115200, SERIAL_8N1, 39, 38);
  encoders.Begin();
  xTaskNotifyGive(led_task_handle);

  struct __attribute__((__packed__)) {
    bool ok;
    float joint_positions[6];
  } tx_message{};

  TickType_t last_send_tick = xTaskGetTickCount();
  while (true) {
    debug_serial.println("===================================");
    encoders.RescanSensors();
    for (int i = 0; i < 6; ++i) {
      auto encoder = encoders[settings::kJointMapping[i]];
      if (encoder == nullptr) {
        debug_serial.println("sensor offline");
        tx_message.joint_positions[i] = 0.f;
        continue;
      }
      tx_message.joint_positions[i] = encoder->readAngle() * AS5600_RAW_TO_RADIANS;
      debug_serial.println(tx_message.joint_positions[i]);
    }
    tx_message.ok = encoders.ok();
    JointPosPostProcess(tx_message.joint_positions);
    for (size_t i = 0; i < 6; ++i) {
      debug_serial.print(tx_message.joint_positions[i]);
      debug_serial.print("\t");
    }
    const auto packet = pb.Build(reinterpret_cast<uint8_t *>(&tx_message), sizeof(tx_message));

    vTaskDelayUntil(&last_send_tick, pdMS_TO_TICKS(33));
    last_send_tick = xTaskGetTickCount();
    rs232.write(packet->data(), packet->size());
  }
}

void setup() {
  debug_serial.begin(921600, SERIAL_8N1, 44, 43);
  xTaskCreate(LedTask, "LedTask", 1024, nullptr, configMAX_PRIORITIES - 2, &led_task_handle);
  xTaskCreate(MainTask, "MainTask", 2048, nullptr, configMAX_PRIORITIES - 1, &main_task_handle);
}

void loop() { vTaskDelay(portMAX_DELAY); }
