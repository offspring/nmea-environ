#include "RgbLed.h"

#include <Arduino.h>

#include <cstdint>

RgbLed::RgbLed(int pin) : _pin(pin), rmt_send(nullptr), led_data() {}

bool RgbLed::begin() {
  rmt_send = rmtInit(_pin, RMT_TX_MODE, RMT_MEM_64);
  if (rmt_send == nullptr) {
    return false;
  }

  rmtSetTick(rmt_send, 100);

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
  if ((rmt_send) == nullptr) {
    return;
  }

  // Color coding is in order RED, REEN, BLUE
  const uint8_t colors[] = {red_val, green_val, blue_val};
  int ii = 0;
  for (const auto& color : colors) {
    for (unsigned int bit = 0; bit < 8; bit++) {
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
  rmtWriteBlocking(rmt_send, led_data, 24);
}
