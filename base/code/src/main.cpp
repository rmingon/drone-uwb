#include <Arduino.h>
#include <LiteLED.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;
volatile boolean receivedAck = false;
#define LEN_DATA 16
byte data[LEN_DATA];

// Comment out all but one LED_TYPE.
#define LED_TYPE        LED_STRIP_WS2812
// #define LED_TYPE        LED_STRIP_SK6812
// #define LED_TYPE        LED_STRIP_APA106
// #define LED_TYPE        LED_STRIP_SM16703

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
void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void setup() {
    Serial.begin(9600);
  Serial.println(F("### DW1000Ng-arduino-receiver-test ###"));
  // initialize the driver
  DW1000Ng::initialize(5, 35, 14);
  Serial.println(F("DW1000Ng initialized ..."));

  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);

  DW1000Ng::setDeviceAddress(6);
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

  DW1000Ng::attachReceivedHandler(handleReceived);

    myLED.begin( LED_GPIO, 1 );         // initialze the myLED object. Here we have 1 LED attached to the LED_GPIO pin
    myLED.brightness( LED_BRIGHT );     // set the LED photon intensity level
    myLED.setPixel( 0, L_GREEN, 1 );    // set the LED colour and show it
    delay( 2000 );
}

void loop() {
    // flash the LED
    myLED.brightness( 0, 1 );           // turn the LED off
    delay( 1000 );

    myLED.brightness( LED_BRIGHT, 1 );  // turn the LED on
    delay( 1000 );

  if (receivedAck) {
    timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
    timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
    timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
    // (re-)compute range as two-way ranging is done
    double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                timePollReceived, 
                                                timePollAckSent, 
                                                timePollAckReceived, 
                                                timeRangeSent, 
                                                timeRangeReceived);
    /* Apply simple bias correction */
    distance = DW1000NgRanging::correctRange(distance);
    
    String rangeString = "Range: "; rangeString += distance; rangeString += " m";
    Serial.println(rangeString);
  }
} 