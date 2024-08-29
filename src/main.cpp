//: Defined in platformio.ini
//: Disable debugs output.
//: #define SERIAL_DEBUG_DISABLED

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <WString.h>
#include <Wire.h>

#ifdef USE_BMP280
#include <Adafruit_BMP280.h>
#else
#include <Adafruit_BME680.h>
#endif

#include <N2kMessages.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <NMEA2000_esp32.h>

#include <ReactESP.h>
#include <esp_log.h>
#include <sensesp/sensors/sensor.h>
#include <sensesp/system/lambda_consumer.h>
#include <sensesp/system/serial_number.h>
#include <sensesp_base_app.h>
#include <sensesp_minimal_app.h>
#include <sensesp_minimal_app_builder.h>

#include "RgbLed.h"

#include <string>

using namespace sensesp;

namespace {

// RGB led pin
#if defined(ESP32_S3_FH4R2)
constexpr gpio_num_t kRgbLedPin = GPIO_NUM_21;
#elif defined(ESP32_S3_DEV_KIT_N8R8)
constexpr gpio_num_t kRgbLedPin = GPIO_NUM_38;
#endif

// I2C pins
constexpr gpio_num_t kSDAPin = GPIO_NUM_2;
constexpr gpio_num_t kSCLPin = GPIO_NUM_1;

// CAN bus (NMEA 2000) pins
constexpr gpio_num_t kCANRxPin = GPIO_NUM_13;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_12;

reactesp::ReactESP app;

// Local name which should pop up when scanning for BLE devices
constexpr char BLE_LOCAL_NAME[]{"BleDemoApp"};

BLEService bleDemoService("fff0");

BLEUnsignedIntCharacteristic rgbLedCharacteristic("fff1", BLERead | BLEWrite);
BLEDescriptor rgbLedDescriptor("2601", "LED RGB Color");

BLEUnsignedIntCharacteristic temperatureCharacteristic("fff2",
                                                       BLERead | BLENotify);
BLEUnsignedIntCharacteristic pressureCharacteristic("fff3",
                                                    BLERead | BLENotify);
BLEUnsignedIntCharacteristic humidityCharacteristic("fff4",
                                                    BLERead | BLENotify);

void make_advertise(float temperatureValue, uint32_t pressureValue,
                    float humidityValue) {
  // Build advertising data packet
  BLEAdvertisingData advData;

  advData.setFlags(BLEFlagsGeneralDiscoverable | BLEFlagsBREDRNotSupported);

  uint8_t mfg_data[12];
  const uint32_t temp = static_cast<uint32_t>(temperatureValue * 100.0);
  memcpy(&mfg_data[0], &temp, sizeof(temp));
  memcpy(&mfg_data[4], &pressureValue, sizeof(pressureValue));
  const uint32_t humidity = static_cast<uint32_t>(humidityValue * 100.0);
  memcpy(&mfg_data[8], &humidity, sizeof(humidity));

  advData.setManufacturerData(0x02E1, mfg_data, sizeof(mfg_data));

  // Copy set parameters in the actual advertising packet
  BLE.setAdvertisingData(advData);

  // Build scan response data packet
  BLEAdvertisingData scanData;
  scanData.setLocalName(BLE_LOCAL_NAME);
  BLE.setDeviceName(BLE_LOCAL_NAME);
  // Copy set parameters in the actual scan response packet
  BLE.setScanResponseData(scanData);

  // start advertising
  if (0 == BLE.advertise()) {
    Serial.println(("Bluetooth® advertise failed"));
    BLE.stopAdvertise();
  } else {
    Serial.println(("Bluetooth® advertise started"));
  }
}

}  // namespace

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  Serial.begin(115200);
  // while (!Serial) {
  //   /*no op*/
  // }
  SetupLogging(ESP_LOG_WARN);
#endif

  // initialize the I2C bus
  auto *i2c = new TwoWire(0);
  if (!i2c->begin(kSDAPin, kSCLPin)) {
    log_e("TwoWire initialization failed!");
  }

  auto *rgbLed = new RgbLed(kRgbLedPin);
  if (!rgbLed->begin()) {
    log_e("RGB LED driver initialization failed!");
  }
#ifdef USE_BMP280
  auto *bmp280 = new Adafruit_BMP280(i2c);
  // 0x60 for BME280
  if (!bmp280->begin(BMP280_ADDRESS_ALT, 0x60)) {
    log_e("Adafruit BMP280 driver initialization failed!");
  }
#else
  auto *bmp680 = new Adafruit_BME680(i2c);
  // 0x60 for BME280
  if (!bmp680->begin(BME68X_DEFAULT_ADDRESS)) {
    log_e("Adafruit BME680 driver initialization failed!");
  }
  // Set up oversampling and filter initialization
  bmp680->setTemperatureOversampling(BME680_OS_8X);
  bmp680->setHumidityOversampling(BME680_OS_2X);
  bmp680->setPressureOversampling(BME680_OS_4X);
  bmp680->setIIRFilterSize(BME680_FILTER_SIZE_3);
  bmp680->setGasHeater(320, 150);  // 320*C for 150 ms
#endif

  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getChipModel(),
                ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());

  /////////////////////////////////////////////////////////////////////
  // RDB led blinker for testing

  static uint32_t rgbLedColor = 0x040404;

  reactesp::ReactESP::app->onRepeat(500, [rgbLed]() {
    static int blue_led_state = 0;
    if (0 == (blue_led_state++ & 1)) {
      rgbLed->set(rgbLedColor);
    } else {
      rgbLed->set(0);
    }
  });

  /////////////////////////////////////////////////////////////////////
  // Read analog values

  static float temperatureValue = 0;
  static uint32_t pressureValue = 0;
  static float humidityValue = 0;
  static uint32_t gasResistanceValue = 0;

#ifdef USE_BMP280
  reactesp::ReactESP::app->onRepeat(10000, [bmp280]() {
    temperatureValue = bmp280->readTemperature();
    pressureValue = bmp280->readPressure();
    humidityValue = random(10000, 50000);
  });
#else
  reactesp::ReactESP::app->onRepeat(10000, [bmp680]() {
    if (!bmp680->performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    Serial.printf("DDDD t=%f p=%d h=%f g=%d\n", bmp680->temperature,
                  bmp680->pressure, bmp680->humidity, bmp680->gas_resistance);

    // Temperature (Celsius)
    temperatureValue = bmp680->temperature;
    // Pressure (Pascals)
    pressureValue = bmp680->pressure;
    // Humidity (RH %)
    humidityValue = bmp680->humidity;
    // Gas resistor (ohms)
    gasResistanceValue = bmp680->gas_resistance;
  });
#endif

  /////////////////////////////////////////////////////////////////////
  // BLE

  if (0 == BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    for (;;) {
      /*no op*/
    }
  }
  Serial.println("BT init");

  delay(200);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, [](BLEDevice central) {
    // central connected event handler
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
  });
  BLE.setEventHandler(BLEDisconnected, [](BLEDevice central) {
    // central disconnected event handler
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
  });

  // assign event handlers for characteristic
  rgbLedCharacteristic.addDescriptor(rgbLedDescriptor);
  rgbLedCharacteristic.setEventHandler(
      BLEWritten,
      [](BLEDevice central, BLECharacteristic characteristic /*NOLINT*/) {
        (void)central;
        Serial.print("New rgbLedColor: ");
        characteristic.readValue(rgbLedColor);
        Serial.println(rgbLedColor, HEX);
      });
  // set an initial value for the characteristic
  rgbLedCharacteristic.setValue(rgbLedColor);

  // add the characteristic to the service
  bleDemoService.addCharacteristic(rgbLedCharacteristic);
  bleDemoService.addCharacteristic(pressureCharacteristic);
  bleDemoService.addCharacteristic(temperatureCharacteristic);
  bleDemoService.addCharacteristic(humidityCharacteristic);
  // add service
  BLE.addService(bleDemoService);

  // BLE.setAppearance(0x0300);  // Generic Thermometer
  BLE.setAppearance(0x0540);  // Generic Sensor

  BLE.setPairable(1);  // enable pairing 0=disabled, 1=multi, 2=once

  make_advertise(temperatureValue, pressureValue, humidityValue);

  reactesp::ReactESP::app->onRepeat(10000, []() {
    make_advertise(temperatureValue, pressureValue, humidityValue);
  });

  reactesp::ReactESP::app->onRepeat(10000, []() {
    const BLEDevice central = BLE.central();
    if (central && central.connected()) {
      // Temperature
      temperatureCharacteristic.writeValue(temperatureValue);
      // Pressure
      pressureCharacteristic.writeValue(pressureValue);
      // Humidity
      humidityCharacteristic.writeValue(humidityValue);
    }
  });

  // // Poll for Bluetooth® Low Energy events
  //: every 1 ms
  reactesp::ReactESP::app->onRepeat(1, []() { BLE.poll(); });

  Serial.println(("Bluetooth® device active, waiting for connections..."));

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

  const unsigned long ReceivedMessages[] PROGMEM /* NOLINT */ = {
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
  // NOTE: Used for debugging
  // nmea2000->SetMsgHandler([](const tN2kMsg &N2kMs) { N2kMs.Print(&Serial);
  // });
#endif
  nmea2000->Open();

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

  auto *temperature_sender =
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

  auto *temperature_provider =
      new RepeatSensor<float>(2000, []() { return temperatureValue; });

  temperature_provider->connect_to(temperature_sender);

  auto *pressure_sender = new LambdaConsumer<float>([nmea2000](float pressure) {
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

  auto *pressure_provider =
      new RepeatSensor<float>(2000, []() { return pressureValue; });

  pressure_provider->connect_to(pressure_sender);

  auto *humidity_sender = new LambdaConsumer<float>([nmea2000](float humidity) {
    static unsigned char SensorsSID = 0;
    tN2kMsg N2kMsg;
    SetN2kPGN130313(N2kMsg,
                    SensorsSID,             // SID
                    0,                      // humidityInstance
                    N2khs_OutsideHumidity,  // HumiditySource
                    humidity                // actual humidity
    );
    nmea2000->SendMsg(N2kMsg);
    SensorsSID++;
    if (SensorsSID > 252) {
      SensorsSID = 0;
    }
  });

  auto *humidity_provider =
      new RepeatSensor<float>(2000, []() { return humidityValue; });

  humidity_provider->connect_to(humidity_sender);

  // No need to parse the messages at every single loop iteration
  //: 1 ms will do
  reactesp::ReactESP::app->onRepeat(
      1, [nmea2000]() { nmea2000->ParseMessages(); });
}

void loop() { app.tick(); }
