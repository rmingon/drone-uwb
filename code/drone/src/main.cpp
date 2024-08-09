#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;

#include "config.h"
#include <SPI.h>
#include <LiteLED.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>
#include "Motor.h"

#include <ArduinoJson.h>

Motor motor;

int DISPLAY_VERSION = 3;                            // Which display should be used for the remote controller? 1D or 2D version?

float COMPLEMENTARY_FILTER = 0.80;                   // Complementary filter for combining acc and gyro

double throttle = THROTTLE_MINIMUM + 30;                             // Desired throttle
float angle_desired[3] = {0.0, 0.0, 0.0};           // Desired angle

float gain_p[3] = {3, 3, 3};                   // Gain proportional
float gain_i[3] = {0.02, 0.02, 0.02};                        // Gain integral
float gain_d[3] = {20, 20, 20};                    // Gain derivetive

float filter = 0.9;                                 // Complementary filter for pid

#define PITCH 0                                     // Rotation forward/backward
#define ROLL 1                                      // Rotation left/right
#define YAW 2                                       // Rotation around center

#define MPU_ADDRESS 0x68

/* --- VARIABLES --- */

float error_current[3] = {0, 0, 0};                 // Current error
float error_prev[3] = {0, 0, 0};                    // Previous error

float pid_current[3] = {0, 0, 0};                   // PID weighted (!) sum of proportional, integral and derivitive error
float pid_p[3] = {0, 0, 0};                         // PID proportional error     
float pid_i[3] = {0, 0, 0};                         // PID integral error
float pid_d[3] = {0, 0, 0};                         // PID derivitive error

float angle_current[3];                             // Angle measured after filtering
float angle_acc[3];                                 // Angle measured using accelerometer
float angle_gyro[3];                                // Angle measured using gyro
float angle_acc_offset[3] = {0.5,0.5,0.5};          // Offsets for gyro angle measurement
float angle_gyro_offset[3] = {0.5,0.5,0.5};         // Offsets for acc angle measurement

float angle_acc_raw[3];                             // Accelerator raw data
int16_t angle_gyro_raw[3];                          // Gyro raw data

float time_current;                                 // Current time
float time_prev;                                    // Previous time
double time_elapsed;                                // Elapsed time during the last loop

float rad_to_deg = 180/3.141592654;                 // Constant for convert radian to degrees

float lastCommand = 0;                              // Time when the last comment has been recieved

int sendDataCounter = 0;                            // Counts when data has been sent the last time to reduce amount of data

Adafruit_BMP280 bmp;

uint32_t altitude_forced = 0;
uint32_t altitude = 0;

#define LEN_DATA 16
byte data[LEN_DATA];
uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint16_t replyDelayTimeUS = 3000;

#define LED_TYPE        LED_STRIP_WS2812
#define LED_TYPE_IS_RGBW 0
#define LED_GPIO GPIO_NUM_14
#define LED_BRIGHT 30
static const crgb_t L_RED = 0xff0000;
static const crgb_t L_GREEN = 0x00ff00;
static const crgb_t L_BLUE = 0x0000ff;
static const crgb_t L_WHITE = 0xe0e0e0;

LiteLED myLED( LED_TYPE, LED_TYPE_IS_RGBW );

String uniq = "";

uint8_t battery_pin = 13;

char packetBuffer[2000];

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

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

void getMac() {
  byte mac[6];
  WiFi.macAddress(mac);
  uniq = String(mac[0],HEX) +String(mac[1],HEX) +String(mac[2],HEX) +String(mac[3],HEX) + String(mac[4],HEX) + String(mac[5],HEX);
}

void transmitPosition() {
  DW1000Ng::setTransmitData(uniq);
  DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
  DW1000Ng::clearTransmitStatus();
}

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int newx;
int newy;
int newz;
double x;
double y;
double z;

#define X 0
#define Y 1
#define Z 2

void setupMpu6050Registers() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  /* Set gyro's digital low pass filter to ~5Hz */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1A);
  Wire.write(0x06);
  Wire.endTransmission();
}

void calculatePid();
void readGyro();
void readAccelerometer();
void filterAngle();
void calculatePid();
void setMotorPids();
void receiveControl();
void setMotorPids();
void sendData2();
void emergencyLanding();
void sendData1(int);
void calibrateAngleOffsets();

void sendDataToServer(String type, JsonDocument data) {
  JsonDocument doc;
  doc["uniq"] = uniq;
  doc["type"] = type;
  doc["data"] = data;
  udp.beginPacket(SERVER_HOST_NAME, UDP_PORT);
  serializeJson(doc, udp);
  udp.endPacket();
}

void sendPositionToServer() {
  JsonDocument doc;
  JsonDocument position;
  
  position["pitch"] = angle_current[PITCH];
  position["roll"] = angle_current[ROLL];
  position["yaw"] = angle_current[YAW];
  position["throttle"] = throttle;

  doc["uniq"] = uniq;
  doc["type"] = "position";
  doc["data"] = position;
  udp.beginPacket(SERVER_HOST_NAME, UDP_PORT);
  serializeJson(doc, udp);
  udp.endPacket();
}

#include <ESP32Servo.h>
Servo esc1, esc2, esc3, esc4;

void calibMotor() {
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  esc1.setPeriodHertz(200);
  esc1.attach(25, 1000, 2000);

  esc2.setPeriodHertz(200);
  esc2.attach(26, 1000, 2000);

  esc3.setPeriodHertz(200);
  esc3.attach(27, 1000, 2000);

  esc4.setPeriodHertz(200);
  esc4.attach(32, 1000, 2000);

  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);

  delay(2000);

  esc1.write(255);
  esc2.write(255);
  esc3.write(255);
  esc4.write(255);

  delay(5000);

  esc1.write(10);
  esc2.write(10);
  esc3.write(10);
  esc4.write(10);

  delay(5000);

  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);

  delay(30000);
}
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  
  delay(2000);

  motor.init(THROTTLE_MINIMUM, THROTTLE_MAXIMUM);

  delay(300);

  motor.arm();
  
  delay(1000);
  
  motor.test();

  Serial.begin(115200);

  myLED.begin( LED_GPIO, 1 );         // initialze the myLED object. Here we have 1 LED attached to the LED_GPIO pin
  myLED.brightness( LED_BRIGHT );     // set the LED photon intensity level

  getMac();

  Wire.begin(21, 22, 400000);
  setupMpu6050Registers();
  calibrateAngleOffsets();

  pinMode(battery_pin, INPUT_PULLUP);

  DW1000Ng::initializeNoInterrupt(5, 14);
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::setNetworkId(10);
  DW1000Ng::setDeviceAddress(6);
  DW1000Ng::setAntennaDelay(16436);

  transmitPosition();

/*
  if (bmp.takeForcedMeasurement()) {
    altitude = bmp.readAltitude(1013.25);
    Serial.println(altitude);
  }
*/


  WiFi.begin(SSID, PASSWORD);
  for(int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
      myLED.setPixel( 0, L_RED, 1 );
      Serial.print(".");
      delay(1000);
  }

  udp.begin(UDP_PORT);
  JsonDocument data;
  sendDataToServer("drone", data);

  myLED.setPixel( 0, L_GREEN, 1 );
}

void loop() {
  time_prev = time_current;
  time_current = millis();
  time_elapsed = (time_current - time_prev) / 1000;

  /* Get gyro's data */
  readGyro();
  readAccelerometer();

  // transmitPosition();
  /* Filter the data to reduce noise */
  filterAngle();

  /* Receive the remote controller's commands */
  // receiveControl();

  /* Calculate PID */
  calculatePid();

  setMotorPids();

  sendData2();

  // sendPositionToServer();
}

void calculatePid() {
  /* Save previous errors */
  error_prev[PITCH] = error_current[PITCH];
  error_prev[ROLL] = error_current[ROLL];
  error_prev[YAW] = error_current[YAW];

  /* Calculate current error */
  error_current[PITCH] = angle_current[PITCH] - angle_desired[PITCH];
  error_current[ROLL] = angle_current[ROLL] - angle_desired[ROLL];
  error_current[YAW] = angle_current[YAW] - angle_desired[YAW];

  /* Calculate weighted proportional error */
  pid_p[PITCH] = gain_p[PITCH] * error_current[PITCH];
  pid_p[ROLL] = gain_p[ROLL] * error_current[ROLL];
  pid_p[YAW] = gain_p[YAW] * error_current[YAW];

  /* Calculated weighted derivitive error */
  float pid_d_new[3];

  pid_d_new[PITCH] = gain_d[PITCH] * (error_current[PITCH] - error_prev[PITCH]) / time_elapsed;
  pid_d_new[ROLL] = gain_d[ROLL] * (error_current[ROLL] - error_prev[ROLL]) / time_elapsed;
  pid_d_new[YAW] = gain_d[YAW] * (error_current[YAW] - error_prev[YAW]) / time_elapsed;

  pid_d[PITCH] = filter * pid_d[PITCH] + (1 - filter) * pid_d_new[PITCH];
  pid_d[ROLL] = filter * pid_d[ROLL] + (1 - filter) * pid_d_new[ROLL];
  pid_d[YAW] = filter * pid_d[YAW] + (1 - filter) * pid_d_new[YAW];

  /* Calculate weighted sum of the PID */
  pid_current[PITCH] = pid_p[PITCH] + pid_i[PITCH] + pid_d[PITCH];
  pid_current[ROLL] = pid_p[ROLL] + pid_i[ROLL] + pid_d[ROLL];
  pid_current[YAW] = pid_p[YAW] + pid_i[YAW] + pid_d[YAW];
}

void setMotorPids() {
    motor.frontRight(throttle + pid_current[PITCH] + pid_current[ROLL] + pid_current[YAW]);      // Set PID for back left motor
    motor.frontLeft(throttle + pid_current[PITCH] - pid_current[ROLL] - pid_current[YAW]);      // Set PID for back left motor
    motor.rearLeft(throttle - pid_current[PITCH] - pid_current[ROLL] + pid_current[YAW]);      // Set PID for back right motor
    motor.rearRight(throttle - pid_current[PITCH] + pid_current[ROLL] - pid_current[YAW]);      // Set PID for back right motor
}

void emergencyLanding() {
  throttle = throttle - 0.25;
      
  angle_desired[0] = 0.0;
  angle_desired[1] = 0.0;
  angle_desired[2] = 0.0;
}

void filterAngle() {
  float angle_new[3];

  angle_new[PITCH] = -(COMPLEMENTARY_FILTER * (-angle_current[PITCH] + angle_gyro[PITCH] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[PITCH]);    // Positive angle -> forward
  angle_new[ROLL] = COMPLEMENTARY_FILTER * (angle_current[ROLL] + angle_gyro[ROLL] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[ROLL];            // Positive angle -> right
  angle_new[YAW] = COMPLEMENTARY_FILTER * (angle_current[YAW] + angle_gyro[YAW] * time_elapsed) + (1 - COMPLEMENTARY_FILTER) * angle_acc[YAW];                // Calculated by chris

  float value = 0.5; // some weird stuff is going on here, this is we we use the value variable

  angle_current[PITCH] = value * angle_current[PITCH] + (1 - value) * angle_new[PITCH];
  angle_current[ROLL] = value * angle_current[ROLL] + (1 - value) * angle_new[ROLL];
  angle_current[YAW] = value * angle_current[YAW] + (1 - value) * angle_new[YAW];
}

void readGyro() {
  /* Ask gyro for gyro data */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  /* Save received answer */
  angle_gyro_raw[PITCH] = Wire.read()<<8|Wire.read();
  angle_gyro_raw[ROLL] = Wire.read()<<8|Wire.read();
  angle_gyro_raw[YAW] = Wire.read()<<8|Wire.read();         // Added, did not check if that works

  /* Convert the data to degrees */
  angle_gyro[PITCH] = angle_gyro_raw[PITCH] / 131.0;
  angle_gyro[ROLL] = angle_gyro_raw[ROLL] / 131.0;
  angle_gyro[YAW] = angle_gyro_raw[YAW] / 131.0;              // Added, did not check if that works

  /* Subtract gyro offset value, this is done here, because the total angle is calculated by integration */
  angle_gyro[PITCH] = angle_gyro[PITCH] - angle_gyro_offset[PITCH];
  angle_gyro[ROLL] = angle_gyro[ROLL] - angle_gyro_offset[ROLL];
  angle_gyro[YAW] = angle_gyro[YAW] - angle_gyro_offset[YAW];
}

void readAccelerometer() {
  /* Ask gyro for acceleration data */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  /* Save received answer */
  angle_acc_raw[PITCH] = (Wire.read()<<8|Wire.read()) / 16384.0;
  angle_acc_raw[ROLL] = (Wire.read()<<8|Wire.read()) / 16384.0;
  angle_acc_raw[YAW] = (Wire.read()<<8|Wire.read()) / 16384.0;

  /* Convert the data to g */
  angle_acc[PITCH] = atan(angle_acc_raw[ROLL] / sqrt(pow(angle_acc_raw[PITCH], 2) + pow(angle_acc_raw[YAW], 2))) * rad_to_deg;
  angle_acc[ROLL] = atan(-1 * angle_acc_raw[PITCH] / sqrt(pow(angle_acc_raw[ROLL], 2) + pow(angle_acc_raw[YAW], 2))) * rad_to_deg;
  angle_acc[YAW] = atan(angle_acc_raw[PITCH] / angle_acc_raw[ROLL]);
  //angle_acc[YAW] = angle_acc_raw[YAW] / 16384.0;                   // Calculated by my own, don't know if its correct...

  /* Subtract Acc angle offsets, this is done here, since the total angle is calculated by integration and offsets there interfere with integration */
  angle_acc[PITCH] = angle_acc[PITCH] - angle_acc_offset[PITCH];
  angle_acc[ROLL] = angle_acc[ROLL] - angle_acc_offset[ROLL];
  angle_acc[YAW] = angle_acc[YAW] - angle_acc_offset[YAW];
}

void calibrateAngleOffsets() {
  int num = 200;
  float gyro_avg[3] = {0.0,0.0,0.0};
  float acc_avg[3] = {0.0,0.0,0.0};
  for(int i = 0; i < num; i++){
    // Read Gyro angles and add to average
    readGyro();
    gyro_avg[PITCH] += angle_gyro[PITCH];
    gyro_avg[ROLL] += angle_gyro[ROLL];
    gyro_avg[YAW] += angle_gyro[YAW];
    
    // Read ACC angles and add to average
    readAccelerometer();
    acc_avg[PITCH] += angle_acc[PITCH];
    acc_avg[ROLL] += angle_acc[ROLL];
    acc_avg[YAW] += angle_acc[YAW];
  }

  // divide sums by number of measurements to get average
  angle_gyro_offset[PITCH] = gyro_avg[PITCH] / num;
  angle_gyro_offset[ROLL] = gyro_avg[ROLL] / num;
  angle_gyro_offset[YAW] = gyro_avg[YAW] / num;
  angle_acc_offset[PITCH] = acc_avg[PITCH] / num;
  angle_acc_offset[ROLL] = acc_avg[ROLL] / num;
  angle_acc_offset[YAW] = acc_avg[YAW] / num;
}

void receiveControl() {

  int packetSize = udp.parsePacket();

  if (packetSize) {
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

    throttle = doc["t"];
    angle_desired[PITCH] = doc["p"];
    angle_desired[ROLL] = doc["r"];
    angle_desired[YAW] = doc["y"];
  }



/*
  if(Serial.available()) {
    // Read until end of command
    String command = Serial.readStringUntil(';');

    if(command[0] == '.') {
      // Receive direct value command

      if(command[1] == 't') {
        throttle = command.substring(2).toInt();                    // Set throttle to value
      } else if(command[1] == 'p') {
        angle_desired[PITCH] = command.substring(2).toFloat();      // Set desired PITCH
      } else if(command[1] == 'r') {
        angle_desired[ROLL] = command.substring(2).toFloat();       // Set Desired ROLL
      } else if(command[1] == 'y') {
        angle_desired[YAW] = command.substring(2).toFloat();        // Set desired YAW
      }

      lastCommand = millis();
    } else {
      // Receive increase/decrease command

      if(command == "throttle+") {
        throttle += 50;                                             // Increase throttle

        if(throttle >= THROTTLE_MAXIMUM) {
          throttle = THROTTLE_MAXIMUM;
        }
      } else if(command == "throttle-") {
        throttle -= 50;                                             // Decrease throttle

        if(throttle <= THROTTLE_MINIMUM) {
          throttle = THROTTLE_MINIMUM;
        }
      } else if(command == "stop") {
        throttle = THROTTLE_MINIMUM;                                // Turn off all motors
      } else if(command == "calibrateAngles") {
        calibrateAngleOffsets();                                    // Calibrate gyro and accelerometer offsets
      } else if(command == "gainP+") {
        gain_p[PITCH] = gain_p[PITCH] + 0.1;                        // Increase P gain for pitch
        gain_p[ROLL] = gain_p[ROLL] + 0.1;                          // Increase P gain for roll
      } else if(command == "gainP-") {
        gain_p[PITCH] = gain_p[PITCH] - 0.1;                        // Decrease P gain for pitch
        gain_p[ROLL] = gain_p[ROLL] - 0.1;                          // Decrease P gain for roll
      } else if(command == "gainD+") {
        gain_d[PITCH] = gain_d[PITCH] + 0.05;                       // Increase D gain for pitch
        gain_d[ROLL] = gain_d[ROLL] + 0.05;                         // Increase D gain for roll
      } else if(command == "gainD-") {
        gain_d[PITCH] = gain_d[PITCH] - 0.05;                       // Decrease D gain for pitch
        gain_d[ROLL] = gain_d[ROLL] - 0.05;                         // Decrease D gain for roll
      } else if(command == "right") {
        angle_desired[ROLL] = angle_desired[ROLL] + 2;              // Move right
      } else if(command == "left") {
        angle_desired[ROLL] = angle_desired[ROLL] - 2;              // Move left
      } else if(command == "filter+") {
        filter = filter + 0.005;                                    // Increase complementary filter
      } else if(command == "filter-") {
        filter = filter - 0.005;                                    // Decrease complementary filter
      } else if(command == "mode0") {
        mode = 0;                                                   // Activate all motors
      } else if(command == "mode1") {
        mode = 1;                                                   // Activate motor 1 & 3
      } else if(command == "mode2") {
        mode = 2;                                                   // Activate motor 2 & 4
      }
    }
  }
*/
}

void sendData1(int angleType) {
  Serial.println("B1" + 
    String(throttle) + "|" + 
    String(angle_current[angleType]) + "|" + 
    String(angle_desired[angleType]) + "|" + 
    String(pid_current[angleType]) + "|" + 
    String(pid_p[angleType]) + "|" + 
    String(pid_d[angleType]) + "|" + 
    String(gain_p[angleType]) + "|" + 
    String(gain_d[angleType], 3) + "|" + 
    String(time_elapsed, 6) + "|" + 
    String(filter, 3) + "|" + 
    String(angle_gyro[angleType], 6) + "|" + 
    String(angle_acc[angleType], 6));
}

void sendData2() {
  Serial.println(throttle + pid_current[PITCH] + pid_current[ROLL] + pid_current[YAW]);
  Serial.println(throttle + pid_current[PITCH] - pid_current[ROLL] - pid_current[YAW]);
  Serial.println(throttle - pid_current[PITCH] - pid_current[ROLL] + pid_current[YAW]);
  Serial.println(throttle - pid_current[PITCH] + pid_current[ROLL] - pid_current[YAW]);
  Serial.println("B2" +
    String(throttle) + "|" + 
    String(angle_current[PITCH]) + "|" + 
    String(angle_current[ROLL]) + "|" + 
    String(angle_current[YAW]) + "|" + 
    String(angle_desired[PITCH]) + "|" +
    String(angle_desired[ROLL]) + "|" +
    String(pid_current[PITCH]) + "|" + 
    String(pid_current[ROLL]) + "|" + 
    String(gain_p[PITCH]) + "|" + 
    String(gain_d[PITCH], 3) + "|" + 
    String(angle_gyro[PITCH], 6) + "|" + 
    String(angle_gyro[ROLL], 6) + "|" +
    String(angle_acc[PITCH], 6) + "|" +
    String(angle_acc[ROLL], 6) + "E");
}