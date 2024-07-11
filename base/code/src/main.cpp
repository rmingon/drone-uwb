#include <Arduino.h>
#include <LiteLED.h>
#include <SPI.h>
#include <AsyncTCP.h>
#include <WiFi.h>

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

#include "config.h"

uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;

volatile boolean received = false;
volatile boolean error = false;
volatile boolean sentAck = false;
volatile int16_t numReceived = 0;
String message;

int16_t sentNum = 0;

#define LEN_DATA 16
byte data[LEN_DATA];

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

void handleReceived() {
  // status change on reception success
  received = true;
}
void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleError() {
  error = true;
}

void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();
}


static void replyToServer(void* arg) {
	AsyncClient* client = reinterpret_cast<AsyncClient*>(arg);

	// send reply
	if (client->space() > 32 && client->canSend()) {
		char message[32];
		client->add(message, strlen(message));
		client->send();
	}
}

/* event callbacks */
static void handleData(void* arg, AsyncClient* client, void *data, size_t len) {
	Serial.printf("\n data received from %s \n", client->remoteIP().toString().c_str());
	Serial.write((uint8_t*)data, len);

}

void onConnect(void* arg, AsyncClient* client) {
	Serial.printf("\n client has been connected to %s on port %d \n", SERVER_HOST_NAME, TCP_PORT);
	replyToServer(client);
}


void setup() {
  Serial.begin(115200);

  WiFi.begin(SSID, PASSWORD);

  // Wait some time to connect to wifi
  for(int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
      Serial.print(".");
      delay(1000);
  }

	AsyncClient* client = new AsyncClient;
	client->onData(&handleData, client);
	client->onConnect(&onConnect, client);
	client->connect(SERVER_HOST_NAME, TCP_PORT);



  Serial.println(F("### DW1000Ng-arduino-receiver-test ###"));

  DW1000Ng::initialize(5, 35, 14);
  Serial.println(F("DW1000Ng initialized ..."));
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  
  DW1000Ng::setEUI("AA:BB:CC:DD:EE:FF:00:01");

  DW1000Ng::setPreambleDetectionTimeout(64);
  DW1000Ng::setSfdDetectionTimeout(273);
  DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

  DW1000Ng::setNetworkId(RTLS_APP_ID);
  DW1000Ng::setDeviceAddress(1);

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

  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);

  receiver();

  myLED.begin( LED_GPIO, 1 );         // initialze the myLED object. Here we have 1 LED attached to the LED_GPIO pin
  myLED.brightness( LED_BRIGHT );     // set the LED photon intensity level
  myLED.setPixel( 0, L_GREEN, 1 );    // set the LED colour and show it

}

void loop() {
    // flash the LED
    /*
    myLED.brightness( 0, 1 );
    delay( 1000 );

    myLED.brightness( LED_BRIGHT, 1 ); 
    delay( 1000 );
    */

  if (received) {
    numReceived++;
    // get data as string
    Serial.print("Received message ... #"); Serial.println(numReceived);
    Serial.print("Data is ... "); Serial.println(message);
    received = false;
  }
  if (error) {
    Serial.println("Error receiving a message");
    error = false;
    Serial.print("Error data is ... "); Serial.println(message);
  }
} 

