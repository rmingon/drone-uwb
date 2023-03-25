#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>

//const uint8_t PIN_RST = 9; // reset pin
//const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// DEBUG packet sent status and count
volatile unsigned long delaySent = 0;
int16_t sentNum = 0; // todo check int type

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
  delay(1000);
  DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
  delaySent = millis();
  while(!DW1000Ng::isTransmitDone()) {}
  sentNum++;
  DW1000Ng::clearTransmitStatus();
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("### DW1000Ng-arduino-sender-test ###"));
  DW1000Ng::initializeNoInterrupt(PIN_SS);
  Serial.println(F("DW1000Ng initialized ..."));

  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
	//DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

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

  transmit();
}

void loop() {
    transmit();
    Serial.print("ARDUINO delay sent [ms] ... "); Serial.println(millis() - delaySent);
    uint64_t newSentTime = DW1000Ng::getTransmitTimestamp();
    Serial.print("Processed packet ... #"); Serial.println(sentNum);
}