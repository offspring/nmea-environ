//: Defined in platformio.ini
//: Disable debugs output.
//: #define SERIAL_DEBUG_DISABLED

#include <Arduino.h>
#include <WString.h>
#include <Wire.h>

#include <Adafruit_BMP280.h>

#include <N2kMessages.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <NMEA2000_esp32.h>

#include <esp_log.h>
#include <ReactESP.h>
#include <sensesp/sensors/sensor.h>
#include <sensesp/system/lambda_consumer.h>
#include <sensesp/system/serial_number.h>
#include <sensesp_base_app.h>
#include <sensesp_minimal_app.h>
#include <sensesp_minimal_app_builder.h>

#include "RgbLed.h"

using namespace sensesp;

namespace {

// RGB led pin
constexpr gpio_num_t kRgbLedPin = GPIO_NUM_21;

// I2C pins
constexpr gpio_num_t kSDAPin = GPIO_NUM_2;
constexpr gpio_num_t kSCLPin = GPIO_NUM_1;

// CAN bus (NMEA 2000) pins
constexpr gpio_num_t kCANRxPin = GPIO_NUM_13;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_12;

reactesp::ReactESP app;

}  // namespace

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupLogging(ESP_LOG_WARN);
#endif

  // initialize the I2C bus
  auto *i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  auto *rgbLed = new RgbLed(kRgbLedPin);
  if (!rgbLed->begin()) {
    log_e("RGB LED driver initialization failed!");
  }

  auto *bmp280 = new Adafruit_BMP280(i2c);
  // 0x60 for BME280
  if (!bmp280->begin(BMP280_ADDRESS_ALT, 0x60)) {
    log_e("Adafruit BMP280 driver initialization failed!");
  }

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  tNMEA2000 *nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20240801",             // Manufacturer's Model serial code (max 32 chars)
      103,                    // Manufacturer's product code
      "ExternalEnvironment",  // Manufacturer's Model ID (max 33 chars)
      "2024.08.01",  // Manufacturer's Software version code (max 40 chars)
      "20240801"     // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  //: http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
  //: https://manualzz.com/doc/12647142/nmea2000-class-and-function-codes

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      85,                      // Device function=External environment
      130,                     // Device class=Atmospheric
      2024                     // Manufacture code
  );

  const unsigned long ReceivedMessages[] PROGMEM = {
      126992L,  // System Time
      129033L,  // Local Time Offset
      0         // End of list
  };
  nmea2000->ExtendReceiveMessages(ReceivedMessages);

  // Initial N2k node address
  // NOTE: For certified NMEA 2000 devices it is mandatory save changed
  // address to e.g. EEPROM, for use in next startup.
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 72);
  nmea2000->EnableForward(false);
#ifndef SERIAL_DEBUG_DISABLED
#if 1  // NOTE: Used for debugging
  nmea2000->SetMsgHandler([](const tN2kMsg &N2kMs) { N2kMs.Print(&Serial); });
#endif
#endif
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will
  // do
  reactesp::ReactESP::app->onRepeat(
      1, [nmea2000]() { nmea2000->ParseMessages(); });

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  SensESPMinimalAppBuilder builder;
  SensESPMinimalApp *sensesp_app =
      (&builder)
          // EDIT: Set a custom hostname for the app.
          ->set_hostname("ExternalEnvironment")
          ->get_app();

  // 0x1FD08: PGN 130312 - Temperature
  // 0x1FD09: PGN 130313 - Humidity
  // 0x1FD0A: PGN 130314 - Actual Pressure
  // 0x1FD0B: PGN 130315 - Set Pressure
  // 0x1FD0C: PGN 130316 - Temperature Extended Range

  // Implement the N2K PGN sending.
  auto *temperatue_sender =
      new LambdaConsumer<float>([nmea2000](float temperature) {
        static unsigned char SensorsSID = 0;
        tN2kMsg N2kMsg;
        SetN2kPGN130316(N2kMsg,
                        SensorsSID,                // SID
                        0,                         // TempInstance
                        N2kts_OutsideTemperature,  // TempSource
                        CToKelvin(temperature)     // actual temperature
        );
        nmea2000->SendMsg(N2kMsg);
        SensorsSID++;
        if (SensorsSID > 252) {
          SensorsSID = 0;
        }
      });

  auto *temperature_provider = new RepeatSensor<float>(
      2000, [bmp280]() { return bmp280->readTemperature(); });

  temperature_provider->connect_to(temperatue_sender);

  auto *pressure_sender =
      new LambdaConsumer<float>([nmea2000](float pressure) {
        static unsigned char SensorsSID = 0;
        tN2kMsg N2kMsg;
        SetN2kPGN130314(N2kMsg,
                        SensorsSID,         // SID
                        0,                  // PressureInstance
                        N2kps_Atmospheric,  // PressureSource
                        pressure            // actual pressure
        );
        nmea2000->SendMsg(N2kMsg);
        SensorsSID++;
        if (SensorsSID > 252) {
          SensorsSID = 0;
        }
      });

  auto *pressure_provider = new RepeatSensor<float>(
      2000, [bmp280]() { return bmp280->readPressure(); });

  pressure_provider->connect_to(pressure_sender);

  /////////////////////////////////////////////////////////////////////
  // RDB led blinker for testing

  reactesp::ReactESP::app->onRepeat(1000, [rgbLed]() {
    static int blue_led_state = 0;
    switch (blue_led_state) {
      default:
        blue_led_state = 0;
        // fallthrough
      case 0:
        rgbLed->set(0x000000);  // off
        break;
      case 1:
        rgbLed->set(0x040404);  // white
        break;
      case 2:
        rgbLed->set(0x040000);  // red
        break;
      case 3:
        rgbLed->set(0x000400);  // green
        break;
      case 4:
        rgbLed->set(0x000004);  // blue
        break;
    }
    blue_led_state++;
  });
}

void loop() { app.tick(); }
