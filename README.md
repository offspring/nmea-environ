# NMEA 2000 Environment sensor

Verified using

- [waveshare esp32-s3-tiny](https://www.waveshare.com/esp32-s3-tiny.htm)
- [waveshare sn65hvd230-can-board](https://www.waveshare.com/sn65hvd230-can-board.htm)
- [waveshare bme280-environmental-sensor](https://www.waveshare.com/bme280-environmental-sensor.htm)

## clang-tidy

Make sure extensio `ms-vscode.cpptools` is installed.

There is no way to exclude includes as of current version of llvm tools.

There is also no way to exclude system headers in `platformio` as the don't use system headers argumnents.

A lot of warning can be ignored as they are very pedantic.

You may need a board definition [platformio_boards](https://github.com/offspring/platformio_boards)
