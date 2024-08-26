/*
  Single RGB LED Controller based on WS2812
*/
#ifndef _RGBLED_H_
#define _RGBLED_H_

#include <Arduino.h>

#include <cstdint>

class RgbLed {
 private:
  int _pin;
  rmt_obj_t *rmt_send;
  rmt_data_t led_data[24];

 public:
  RgbLed(int pin);

  bool begin();

  void set(uint32_t color);

  void write(uint8_t red_val, uint8_t green_val, uint8_t blue_val);
};

#endif
