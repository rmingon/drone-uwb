#include <Arduino.h>
#include <LiteLED.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

#include "config.h"

#include <ArduinoJson.h>

int16_t sentNum = 0;

#define LEN_DATA 16
byte data[LEN_DATA];
String message;

#define LED_TYPE        LED_STRIP_WS2812
#define LED_TYPE_IS_RGBW 0   // if the LED is an RGBW type, change the 0 to 1
#define LED_GPIO 17     // change this number to be the GPIO pin connected to the LED
#define LED_BRIGHT 30   // sets how bright the LED is. O is off; 255 is burn your eyeballs out (not recommended)

// pick the colour you want from the list here and change it in setup()
static const crgb_t L_RED = 0xff0000;
static const crgb_t L_GREEN = 0x00ff00;
static const crgb_t L_BLUE = 0x0000ff;
static const crgb_t L_WHITE = 0xe0e0e0;
LiteLED myLED( LED_TYPE, LED_TYPE_IS_RGBW );    // create the LiteLED object; we're calling it "myLED"

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

String uniq = "";

float samplingRate = 0;
int accuracyCounter = 0;
uint16_t antenna_delay = 16436;

char packetBuffer[255];

void receiver() {
    DW1000Ng::forceTRxOff();
    DW1000Ng::startReceive();
}

void transmit() {
  DW1000Ng::setTransmitData("A"+uniq);
  delay(200);
  DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
  delay(200);
  DW1000Ng::clearTransmitStatus();
  delay(200);
  DW1000Ng::startReceive();
}

void sendDataToServer(String type, JsonDocument data) {
  JsonDocument doc;
  doc["uniq"] = uniq;
  doc["type"] = type;
  doc["data"] = data;
  udp.beginPacket(SERVER_HOST_NAME, UDP_PORT);
  serializeJson(doc, udp);
  udp.endPacket();
}

void getMac() {
  byte mac[6];
  WiFi.macAddress(mac);
  uniq = String(mac[0],HEX) +String(mac[1],HEX) +String(mac[2],HEX) +String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX);
}

void setup() {
  Serial.begin(115200);

  getMac();

  WiFi.begin(SSID, PASSWORD);

  myLED.begin( LED_GPIO, 2 ); 
  myLED.brightness( LED_BRIGHT );    

  for(int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
      myLED.setPixel( 0, L_RED, 1 );
      Serial.print(".");
      delay(1000);
  }
  myLED.setPixel( 0, L_BLUE, 1 );

  udp.begin(UDP_PORT);

  JsonDocument data;
  sendDataToServer("anchor", data);

  Serial.println(F("### DW1000Ng-arduino-receiver-test ###"));
  // initialize the driver
  DW1000Ng::initializeNoInterrupt(5, 14);
  Serial.println(F("DW1000Ng initialized ..."));

  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);

  DW1000Ng::setDeviceAddress(6);
  DW1000Ng::setNetworkId(10);

  DW1000Ng::setAntennaDelay(16436);
  Serial.println(F("Committed configuration ..."));

  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);

  delay(1000);
  
  transmit();
  
  myLED.setPixel( 0, L_GREEN, 1 );
}

void loop() {

  int packetSize = udp.parsePacket();

  if (packetSize) {
    Serial.print(" Received packet from : "); Serial.println(udp.remoteIP());
    Serial.print(" Size : "); Serial.println(packetSize);
    int len = udp.read(packetBuffer, 255);
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, packetBuffer);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    if (doc["reboot"]) {
      ESP.restart();
    }
  } 
  
  if(DW1000Ng::isReceiveDone()) {
    myLED.setPixel( 1, L_GREEN, 1 );
    DW1000Ng::getReceivedData(message);
    JsonDocument data;
    data["uniq"] = message;
    data["quality"] = DW1000Ng::getReceiveQuality();
    data["power"] = DW1000Ng::getReceivePower();
    sendDataToServer("range", data);
    DW1000Ng::startReceive();
    myLED.setPixel( 1, L_RED, 1 );
  }

} 

