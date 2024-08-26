#include "RgbLed.h"

#include <Arduino.h>

#include <cstdint>

RgbLed::RgbLed(int pin) : _pin(pin), _rmt_send(nullptr) {}

bool RgbLed::begin() {
  _rmt_send = rmtInit(_pin, RMT_TX_MODE, RMT_MEM_64);
  if (nullptr == _rmt_send) {
    return false;
  }

  rmtSetTick(_rmt_send, 100);

  set(0x010101);  // white
  delay(100);
  set(0x000000);  // off
  delay(100);

  return true;
}

void RgbLed::set(uint32_t color) {
  RgbLed::write((color & 0xff0000) >> 16, (color & 0x00ff00) >> 8,
                (color & 0x0000ff));
}

void RgbLed::write(uint8_t red_val, uint8_t green_val, uint8_t blue_val) {
  if (nullptr == _rmt_send) {
    return;
  }

  int ii = 0;
  rmt_data_t led_data[24];
  // Color coding is in order RED, GREEN, BLUE
  const uint8_t colors[] = {red_val, green_val, blue_val};
  for (const auto& color : colors) {
    for (unsigned int bit = 0; bit < 8; bit++) {
      // from esp32-hal-rgb-led.c
      if (0 != (color & (1 << (7 - bit)))) {
        // HIGH bit
        led_data[ii].level0 = 1;     // T1H
        led_data[ii].duration0 = 8;  // 0.8us
        led_data[ii].level1 = 0;     // T1L
        led_data[ii].duration1 = 4;  // 0.4us
      } else {
        // LOW bit
        led_data[ii].level0 = 1;     // T0H
        led_data[ii].duration0 = 4;  // 0.4us
        led_data[ii].level1 = 0;     // T0L
        led_data[ii].duration1 = 8;  // 0.8us
      }
      ii++;
    }
  }
  rmtWriteBlocking(_rmt_send, led_data, 24);
}
