#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Motor.h>
#include <Adafruit_BMP280.h>
#include <Conf.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "DW1000.h"
#include <NeoPixelBus.h>

#include "WiFi.h"
#include <ArduinoWebsockets.h>

#define ID 1

const char* websockets_server_host = "192.168.1.5"; //Enter server adress
const uint16_t websockets_server_port = 8081; // Enter server port
using namespace websockets;
WebsocketsClient client;

#define SSID "NETGEAR41"
#define PASS "royalgiant308"

const uint16_t PixelCount = 1;
const uint8_t PixelPin = 14; 

NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> strip(PixelCount, PixelPin);

RgbColor red(128, 0, 0);
RgbColor blue(0, 0, 128);
RgbColor green(0, 128, 0);

#define MIN_MOTOR_PWM 20

uint8_t motor_dynamic = 0;

MPU6050 mpu(Wire);

Adafruit_BMP280 bmp;

Motor motor = Motor();

uint32_t altitude_forced = 0;
uint32_t altitude = 0;

boolean fly = false;

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint16_t replyDelayTimeUS = 3000;

uint8_t battery_pin = 13;

void sendUWB();

void noteActivity() {
    lastActivity = millis();
}

void transmitPoll() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void resetInactive() {
    expectedMsgId = POLL_ACK;
    transmitPoll();
    noteActivity();
}

void IRAM_ATTR handleSent() {
    sentAck = true;
}

void IRAM_ATTR handleReceived() {
    receivedAck = true;
}

void transmitRange() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 1);
    timePollAckReceived.getTimestamp(data + 6);
    timeRangeSent.getTimestamp(data + 11);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}



void parseWS(String &data) {
    if (data == "GO") {
      strip.SetPixelColor(0, green);
      strip.Show();
    } else if (data == "F") {
      fly = true;
    } else if (data.substring(0,1) == "M") {
      motor_dynamic = data.substring(1).toInt();
    } else if (data.substring(0,1) == "D") { // DIRECTION + ACCELERATION
      motor_dynamic = data.substring(1).toInt();
    } else if (data == "S") {
      fly = false;
      motor.setPwn(0);
    } else if (data == "R") {
      ESP.restart();
    }
}

void server(String &msg) {
  client.send(String(ID) + msg);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(460800);
  Wire.begin(21, 22, 400000);

  strip.Begin();

  strip.SetPixelColor(0, blue);
  strip.Show();

  motor.init();
  motor.setPwn(0);
  motor.test();

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while(1) {
      delay(500);
    }
  }

  bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
  if(connected) {
    client.send("D"+String(ID));
  } else {
    strip.SetPixelColor(0, red);
    strip.Show();
  }


  client.onMessage([&](WebsocketsMessage message) {
    String data = message.data();
    Serial.println(data);
    parseWS(data);
  });

  pinMode(battery_pin, INPUT_PULLUP);

  mpu.begin();
  mpu.calcOffsets();

  bmp.begin(0x76);
  
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */


  DW1000.begin(16, 33);
  DW1000.select(5);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(2);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  DW1000.getPrintableDeviceMode(msg);
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  receiver();
  transmitPoll();
  noteActivity();

  strip.SetPixelColor(0, green);
  strip.Show();
}

void loop() {

  if(client.available()) {
      client.poll();
  }

  // sendUWB();

  mpu.update();

  int x = map(mpu.getAngleX(), 0, 100, 0, 250);
  int y = map(mpu.getAngleY(), 0, 100, 0, 250);

  server("X"+String(x));
  server("Y"+String(y));
  server("A"+String(bmp.takeForcedMeasurement() * 1000));

  // int alti = map(altitude, altitude_forced-50, altitude_forced+50, 0, 200);

  if (fly) {
    if (-y+x > 0) {
      motor.lf(-y+x + MIN_MOTOR_PWM + motor_dynamic);
    } else {
      motor.lf(MIN_MOTOR_PWM + motor_dynamic);
    }
    if (y+x > 0) {
      motor.lr(y+x + MIN_MOTOR_PWM + motor_dynamic);
    } else {
      motor.lr(MIN_MOTOR_PWM + motor_dynamic);
    }
    if (-y+-x > 0) {
      motor.rl(-y+-x + MIN_MOTOR_PWM + motor_dynamic);
    } else {
      motor.rl(MIN_MOTOR_PWM + motor_dynamic);
    }
    if (y+-x > 0) {
      motor.rr(y+-x + MIN_MOTOR_PWM + motor_dynamic);
    } else {
      motor.rr(MIN_MOTOR_PWM + motor_dynamic);
    }
  }
}

void sendUWB() {
  if (!sentAck && !receivedAck) {
    if (millis() - lastActivity > resetPeriod) {
        resetInactive();
    }
    return;
  }
  if (sentAck) {
      sentAck = false;
      byte msgId = data[0];
      if (msgId == POLL) {
          DW1000.getTransmitTimestamp(timePollSent);
      } else if (msgId == RANGE) {
          DW1000.getTransmitTimestamp(timeRangeSent);
          noteActivity();
      }
  }
  if (receivedAck) {
      receivedAck = false;
      // get message and parse
      DW1000.getData(data, LEN_DATA);
      byte msgId = data[0];
      if (msgId != expectedMsgId) {
          expectedMsgId = POLL_ACK;
          transmitPoll();
          return;
      }
      if (msgId == POLL_ACK) {
          DW1000.getReceiveTimestamp(timePollAckReceived);
          expectedMsgId = RANGE_REPORT;
          transmitRange();
          noteActivity();
      } else if (msgId == RANGE_REPORT) {
          expectedMsgId = POLL_ACK;
          float curRange;
          memcpy(&curRange, data + 1, 4);
          transmitPoll();
          noteActivity();
      } else if (msgId == RANGE_FAILED) {
          expectedMsgId = POLL_ACK;
          transmitPoll();
          noteActivity();
      }
  }

  if (bmp.takeForcedMeasurement()) {
    altitude = bmp.readAltitude(1013.25);
    if (altitude_forced == 0) {
      altitude_forced = altitude;
    }
  }
}