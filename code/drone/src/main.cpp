#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Motor.h>
#include <Adafruit_BMP280.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <NeoPixelBus.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

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

int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

#define MIN_MOTOR_PWM 40
#define MAX_MOTOR_PWM 180

uint8_t motor_dynamic = MIN_MOTOR_PWM;

Adafruit_BMP280 bmp;

Motor motor = Motor();

uint32_t altitude_forced = 0;
uint32_t altitude = 0;

boolean fly = true;

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
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
uint32_t lastActivity;
uint32_t resetPeriod = 250;
uint16_t replyDelayTimeUS = 3000;

uint8_t battery_pin = 13;

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

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    true /* This allows blink frames */
};

void sendUWB();

void noteActivity() {
    lastActivity = millis();
}

void resetInactive() {
    expectedMsgId = POLL_ACK;
    noteActivity();
}

void IRAM_ATTR handleSent() {
    sentAck = true;
}

void IRAM_ATTR handleReceived() {
    receivedAck = true;
}
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

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

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265;
int maxVal=402;

int newx;
int newy;
int newz;
double x;
double y;
double z;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  motor.init();
  motor.setPwn(MAX_MOTOR_PWM);
  delay(2000);
  motor.setPwn(MIN_MOTOR_PWM);

  delay(3000);
  motor.lf(MIN_MOTOR_PWM + 40);
  delay(3000);
  motor.lf(MIN_MOTOR_PWM);
  motor.rl(MIN_MOTOR_PWM + 40);
  delay(3000);
  motor.rl(MIN_MOTOR_PWM);
  motor.rr(MIN_MOTOR_PWM + 40);
  delay(3000);
  motor.rr(MIN_MOTOR_PWM);
  motor.lr(MIN_MOTOR_PWM + 40);
  delay(3000);
  motor.lr(MIN_MOTOR_PWM);


  Serial.begin(115200);
  Wire.begin(21, 22, 400000);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  strip.Begin();

  // strip.SetPixelColor(0, blue);
  // strip.Show();

  pinMode(battery_pin, INPUT_PULLUP);


  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);


  bmp.begin(0x76);
  
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     
                  Adafruit_BMP280::SAMPLING_X1,   
                  Adafruit_BMP280::SAMPLING_X16,  
                  Adafruit_BMP280::FILTER_X16,  
                  Adafruit_BMP280::STANDBY_MS_1);
/*
  DW1000Ng::initializeNoInterrupt(5, 14);
  Serial.println(F("DW1000Ng initialized ..."));
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
  
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
  noteActivity();

  if (bmp.takeForcedMeasurement()) {
    altitude = bmp.readAltitude(1013.25);
    Serial.println(altitude);
  }

  // strip.SetPixelColor(0, green);
  // strip.Show();

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while(1) {
      delay(500);
    }
  }
*/

/*
  bool connected = client.connect(websockets_server_host, websockets_server_port, "/");
  if(connected) {
    client.send("D"+String(ID));
  } else {
    strip.SetPixelColor(0, red);
    strip.Show();
  }
*/


  client.onMessage([&](WebsocketsMessage message) {
    String data = message.data();
    Serial.println(data);
    parseWS(data);
  });

}

void loop() {
  /*
  
  if(client.available()) {
      client.poll();
  }
  */

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-180,0);
  int yAng = map(AcY,minVal,maxVal,-180,0);
  int zAng = map(AcZ,minVal,maxVal,-180,0);
  Serial.print("xAng= ");
  Serial.println(xAng);
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
  newx = x*10;
  newy = y*10;
  newz = z*10;

  Serial.print("X= ");
  Serial.println((float)newx/10,1);
   
  Serial.print("Y= ");
  Serial.println((float)newy/10,1);
   
  Serial.print("Z= ");
  Serial.println((float)newz/10,1);

  int x = map((float)newx/10, 0, 360, MIN_MOTOR_PWM, MAX_MOTOR_PWM);
  int y = map((float)newy/10, 0, 360, MIN_MOTOR_PWM, MAX_MOTOR_PWM);

  Serial.println(x);
  Serial.println(y);

  // server("X"+String(x));
  // server("Y"+String(y));
  // server("A"+String(bmp.takeForcedMeasurement() * 1000));
  if (fly) {
    if (-y+x > 0) {
      motor.lf(-y+x + motor_dynamic);
    } else {
      motor.lf(motor_dynamic);
    }
    if (y+x > 0) {
      motor.lr(y+x + motor_dynamic);
    } else {
      motor.lr(motor_dynamic);
    }
    if (-y+-x > 0) {
      motor.rl(-y+-x + motor_dynamic);
    } else {
      motor.rl(motor_dynamic);
    }
    if (y+-x > 0) {
      motor.rr(y+-x + motor_dynamic);
    } else {
      motor.rr(motor_dynamic);
    }
  }


  // int alti = map(altitude, altitude_forced-50, altitude_forced+50, 0, 200);


}

void sendUWB() {
  if (!sentAck && !receivedAck) {
    if (millis() - lastActivity > resetPeriod) {
        resetInactive();
    }
    return;
  }

  if (bmp.takeForcedMeasurement()) {
    altitude = bmp.readAltitude(1013.25);
    if (altitude_forced == 0) {
      altitude_forced = altitude;
    }
  }
}