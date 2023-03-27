#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Motor.h>
#include <Adafruit_BMP280.h>

MPU6050 mpu(Wire);

Adafruit_BMP280 bmp;

volatile unsigned long delaySent = 0;
int16_t sentNum = 0;

Motor motor = Motor();

device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

void transmit() {
  Serial.print("Transmitting packet ... #"); Serial.println(sentNum);
  String msg = "Hello DW1000Ng, it's #"; msg += sentNum;
  DW1000Ng::setTransmitData(msg);
  DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
  delaySent = millis();
  while(!DW1000Ng::isTransmitDone()) {}
  sentNum++;
  DW1000Ng::clearTransmitStatus();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  mpu.begin();
  mpu.calcOffsets();

  motor.init();

  bmp.begin(0x76);
  
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
                  

  DW1000Ng::initializeNoInterrupt(5);
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);

  DW1000Ng::setDeviceAddress(5);
  DW1000Ng::setNetworkId(10);

  DW1000Ng::setAntennaDelay(16436);
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
}

void loop() {
  transmit();
  Serial.print("ARDUINO delay sent [ms] ... "); Serial.println(millis() - delaySent);
  uint64_t newSentTime = DW1000Ng::getTransmitTimestamp();
  Serial.print("Processed packet ... #"); Serial.println(sentNum);

  mpu.update();
  int x = map(mpu.getAngleX(), 0, 100, 0, 20);
  int y = map(mpu.getAngleY(), 0, 100, 0, 20);
  if (-y+x > 0) {
    motor.lf(-y+x);
  } else {
    motor.lf(0);
  }
  if (y+x > 0) {
    motor.lr(y+x);
  } else {
    motor.lr(0);
  }
  if (-y+-x > 0) {
    motor.rl(-y+-x);
  } else {
    motor.rl(0);
  }
  if (y+-x > 0) {
    motor.rr(y+-x);
  } else {
    motor.rr(0);
  }
  
  if (bmp.takeForcedMeasurement()) {
    // can now print out the new measurements
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
  }
}