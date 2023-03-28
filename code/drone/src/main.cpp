#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Motor.h>
#include <Adafruit_BMP280.h>

#include "DW1000.h"

#define MIN_MOTOR_PWM 5

MPU6050 mpu(Wire);

Adafruit_BMP280 bmp;

Motor motor = Motor();

uint32_t altitude_forced = 0;
uint32_t altitude = 0;

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
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
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
    // tag sends POLL and listens for POLL_ACK
    expectedMsgId = POLL_ACK;
    transmitPoll();
    noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void transmitRange() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;
    // delay sending the message and remember expected future sent timestamp
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 1);
    timePollAckReceived.getTimestamp(data + 6);
    timeRangeSent.getTimestamp(data + 11);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
    //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void setup() {
  Serial.begin(460800);
  Wire.begin(21, 22, 400000);
  
  uint32_t Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);

  mpu.begin();
  mpu.calcOffsets();

  motor.init();

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
}

void loop() {

  if (!sentAck && !receivedAck) {
      // check if inactive
      if (millis() - lastActivity > resetPeriod) {
          resetInactive();
      }
      return;
  }
  // continue on any success confirmation
  if (sentAck) {
      sentAck = false;
      byte msgId = data[0];
      if (msgId == POLL) {
          DW1000.getTransmitTimestamp(timePollSent);
          //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
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

  mpu.update();
  int x = map(mpu.getAngleX(), 0, 100, 0, 200);
  int y = map(mpu.getAngleY(), 0, 100, 0, 200);
  int alti = map(altitude, altitude_forced-50, altitude_forced+50, 0, 200);
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

  Serial.println(alti);
}