
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "AS5600.h"

AS5600 as5600;

void BlinkLedOnceTask(void *arg) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void MainLoop(void *arg) {
  while (true) {
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(as5600.readAngle());
    Serial.print("\t");
    // Serial.print(as5600.rawAngle());
    // Serial.print("\t");
    Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(921600);
  Wire.begin();

  for (int i = 0; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("0x");
      Serial.println(i, HEX);
    }
  }

  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.print("Connect: ");
  Serial.println(as5600.isConnected());
  delay(1000);

  xTaskCreate(MainLoop, "MainLoop", 2048, nullptr, 1, nullptr);
  xTaskCreate(BlinkLedOnceTask, "BlinkLedOnceTask", 1024, nullptr, 1, nullptr);
}

void loop() {}
