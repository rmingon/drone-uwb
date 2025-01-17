#include <Wire.h>
#include "MPU6050.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "secrets.h"

const unsigned int UDP_PORT = 1234; // port to listen on

WiFiUDP udp;                       // UDP instance
char incomingPacket[512];          // buffer to hold incoming packet data

struct Kalman {
  float Q_angle;    // Process noise variance for angle
  float Q_bias;     // Process noise variance for gyro bias
  float R_measure;  // Measurement noise variance

  float angle;      // Filtered angle
  float bias;       // Estimated gyro bias
  float rate;       // Unbiased angular rate

  float P[2][2];    // Error covariance matrix
};

void initKalman(Kalman &kf, float Q_angle, float Q_bias, float R_measure) {
  kf.Q_angle   = Q_angle;
  kf.Q_bias    = Q_bias;
  kf.R_measure = R_measure;

  kf.angle     = 0.0f;
  kf.bias      = 0.0f;
  kf.rate      = 0.0f;

  kf.P[0][0]   = 0.0f;
  kf.P[0][1]   = 0.0f;
  kf.P[1][0]   = 0.0f;
  kf.P[1][1]   = 0.0f;
}

float getKalmanAngle(Kalman &kf, float newAngle, float newRate, float dt) {
  kf.rate   = newRate - kf.bias;
  kf.angle += kf.rate * dt;

  kf.P[0][0] += dt * (dt*kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + kf.Q_angle);
  kf.P[0][1] -= dt * kf.P[1][1];
  kf.P[1][0] -= dt * kf.P[1][1];
  kf.P[1][1] += kf.Q_bias * dt;

  float S  = kf.P[0][0] + kf.R_measure;
  float K0 = kf.P[0][0] / S;
  float K1 = kf.P[1][0] / S;

  float y = newAngle - kf.angle;
  kf.angle += K0 * y;
  kf.bias  += K1 * y;

  float P00_temp = kf.P[0][0];
  float P01_temp = kf.P[0][1];

  kf.P[0][0] -= K0 * P00_temp;
  kf.P[0][1] -= K0 * P01_temp;
  kf.P[1][0] -= K1 * P00_temp;
  kf.P[1][1] -= K1 * P01_temp;

  return kf.angle;
}

// ------------------------------------------------------------------
// PID GAINS (TUNE FOR YOUR DRONE)
// ------------------------------------------------------------------
float Kp_roll  = 1.0f, Ki_roll  = 0.02f, Kd_roll  = 0.05f;
float Kp_pitch = 1.0f, Ki_pitch = 0.02f, Kd_pitch = 0.05f;
float Kp_yaw   = 1.0f, Ki_yaw   = 0.02f, Kd_yaw   = 0.05f;

// ------------------------------------------------------------------
// PID STATE
// ------------------------------------------------------------------
float errorRoll, errorPitch, errorYaw;
float prevErrorRoll  = 0, prevErrorPitch  = 0, prevErrorYaw  = 0;
float integralRoll   = 0, integralPitch   = 0, integralYaw   = 0;

// ------------------------------------------------------------------
// SETPOINTS (Degrees). 0 means level. Could be from RC inputs.
// ------------------------------------------------------------------
float rollSetpoint   = 0.0f;
float pitchSetpoint  = 0.0f;
float yawSetpoint    = 0.0f;

// ------------------------------------------------------------------
// MOTOR OUTPUTS
// ------------------------------------------------------------------
const int motorPin1 = 26;   // front-right OK
const int motorPin2 = 25;   // front-left
const int motorPin3 = 27;   // rear-left
const int motorPin4 = 32;   // rear-right

#define MOTOR_CH_1    0
#define MOTOR_CH_2    1
#define MOTOR_CH_3    2
#define MOTOR_CH_4    3

int baseThrottle = 1200;

int constrainESC(int pwmValue) {
  if (pwmValue < 1000) return 1000;
  if (pwmValue > 2000) return 2000;
  return pwmValue;
}

void armAllESCs();
void calibrateAllESCs();
void writeMotorPWM(int, int);
void readIMUData();

Kalman kalmanRoll;
Kalman kalmanPitch;
Kalman kalmanYaw;

unsigned long currentTime, previousTime;
float elapsedTime; // in seconds

MPU6050 mpu;  

int16_t ax, ay, az;
int16_t gx, gy, gz;

float accRoll  = 0.0f;
float accPitch = 0.0f;
// Yaw from accelerometer is typically unreliable; real code uses magnetometer
float accYaw   = 0.0f; 

float gyroRollRate  = 0.0f;
float gyroPitchRate = 0.0f;
float gyroYawRate   = 0.0f;

float currentRoll  = 0.0f;
float currentPitch = 0.0f;
float currentYaw   = 0.0f;

const int PWM_FREQ    = 50;
const int PWM_RES_BITS = 16;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("\nConnecting to Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());

  // 2. Start UDP
  udp.begin(UDP_PORT);
  Serial.print("UDP listening on port ");
  Serial.println(UDP_PORT);
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }

  ledcSetup(MOTOR_CH_1, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(MOTOR_CH_2, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(MOTOR_CH_3, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(MOTOR_CH_4, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(motorPin1, MOTOR_CH_1);
  ledcAttachPin(motorPin2, MOTOR_CH_2);
  ledcAttachPin(motorPin3, MOTOR_CH_3);
  ledcAttachPin(motorPin4, MOTOR_CH_4);

  initKalman(kalmanRoll,  0.001f, 0.003f, 0.03f);
  initKalman(kalmanPitch, 0.001f, 0.003f, 0.03f);
  initKalman(kalmanYaw,   0.001f, 0.003f, 0.03f);

  previousTime = micros();
  
  // calibrateAllESCs();
  armAllESCs();
}

void loop() {
  currentTime = micros();
  elapsedTime = (float)(currentTime - previousTime) / 1000000.0f;
  previousTime = currentTime;

  readIMUData();

  currentRoll  = getKalmanAngle(kalmanRoll,  accRoll,  gyroRollRate,  elapsedTime);
  currentPitch = getKalmanAngle(kalmanPitch, accPitch, gyroPitchRate, elapsedTime);
  // For yaw, typically you'd rely on a magnetometer or at least a drift compensation technique
  currentYaw   = getKalmanAngle(kalmanYaw,   accYaw,   gyroYawRate,   elapsedTime);

  errorRoll  = rollSetpoint  - currentRoll;
  errorPitch = pitchSetpoint - currentPitch;
  errorYaw   = yawSetpoint   - currentYaw;

  integralRoll  += errorRoll  * elapsedTime;
  integralPitch += errorPitch * elapsedTime;
  integralYaw   += errorYaw   * elapsedTime;

  float derivativeRoll  = (errorRoll  - prevErrorRoll)  / elapsedTime;
  float derivativePitch = (errorPitch - prevErrorPitch) / elapsedTime;
  float derivativeYaw   = (errorYaw   - prevErrorYaw)   / elapsedTime;

  float rollOutput  = Kp_roll  * errorRoll  + Ki_roll  * integralRoll  + Kd_roll  * derivativeRoll;
  float pitchOutput = Kp_pitch * errorPitch + Ki_pitch * integralPitch + Kd_pitch * derivativePitch;
  float yawOutput   = Kp_yaw   * errorYaw   + Ki_yaw   * integralYaw   + Kd_yaw   * derivativeYaw;

  prevErrorRoll  = errorRoll;
  prevErrorPitch = errorPitch;
  prevErrorYaw   = errorYaw;

  int motor1 = baseThrottle + pitchOutput + rollOutput + yawOutput;  // front-right
  int motor2 = baseThrottle + pitchOutput - rollOutput - yawOutput;  // front-left
  int motor3 = baseThrottle - pitchOutput - rollOutput + yawOutput;  // rear-left
  int motor4 = baseThrottle - pitchOutput + rollOutput - yawOutput;  // rear-right

  motor1 = constrainESC(motor1);
  motor2 = constrainESC(motor2);
  motor3 = constrainESC(motor3);
  motor4 = constrainESC(motor4);

  writeMotorPWM(MOTOR_CH_1, motor1);
  writeMotorPWM(MOTOR_CH_2, motor2);
  writeMotorPWM(MOTOR_CH_3, motor3);
  writeMotorPWM(MOTOR_CH_4, motor4);

  // Debug output
  Serial.print("Roll: ");   Serial.print(currentRoll);
  Serial.print("\tPitch: "); Serial.print(currentPitch);
  Serial.print("\tYaw: ");   Serial.print(currentYaw);
  Serial.print("\tM1: ");    Serial.print(motor1);
  Serial.print("\tM2: ");    Serial.print(motor2);
  Serial.print("\tM3: ");    Serial.print(motor3);
  Serial.print("\tM4: ");    Serial.print(motor4);
  Serial.println();

  delay(2);
}

void readIMUData() {
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float A_RES = 16384.0;
  float G_RES = 131.0;

  float Ax = (float)ax / A_RES; // in g
  float Ay = (float)ay / A_RES;
  float Az = (float)az / A_RES;

  // Convert gyro to deg/s
  float Gx = (float)gx / G_RES; 
  float Gy = (float)gy / G_RES; 
  float Gz = (float)gz / G_RES; 

  accPitch = atan2(-Ax, sqrt(Ay*Ay + Az*Az)) * 57.2958;  // 180/PI
  accRoll  = atan2(Ay, Az) * 57.2958;

  accYaw = 0.0f;

  gyroRollRate  = Gx; 
  gyroPitchRate = Gy; 
  gyroYawRate   = Gz;
}

void calibrateAllESCs() {
  Serial.println("CALIBRATING ESCs: Full throttle -> Wait -> Min throttle -> Wait");

  writeMotorPWM(MOTOR_CH_1, 2000);
  writeMotorPWM(MOTOR_CH_2, 2000);
  writeMotorPWM(MOTOR_CH_3, 2000);
  writeMotorPWM(MOTOR_CH_4, 2000);
  delay(2000);

  // Step 2: Min throttle
  writeMotorPWM(MOTOR_CH_1, 1000);
  writeMotorPWM(MOTOR_CH_2, 1000);
  writeMotorPWM(MOTOR_CH_3, 1000);
  writeMotorPWM(MOTOR_CH_4, 1000);
  delay(2000);

  Serial.println("ESC Calibration Complete.");
}

void handleUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Clear buffer
    memset(incomingPacket, 0, sizeof(incomingPacket));

    // Read UDP data
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = 0; // Null-terminate
    }
    Serial.print("[UDP] Received: ");
    Serial.println(incomingPacket);

    // Parse JSON
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, incomingPacket);
    if (!error) {
      // For each possible field, update if present
      if (doc.containsKey("Kp_roll"))   Kp_roll   = doc["Kp_roll"].as<float>();
      if (doc.containsKey("Ki_roll"))   Ki_roll   = doc["Ki_roll"].as<float>();
      if (doc.containsKey("Kd_roll"))   Kd_roll   = doc["Kd_roll"].as<float>();

      if (doc.containsKey("Kp_pitch"))  Kp_pitch  = doc["Kp_pitch"].as<float>();
      if (doc.containsKey("Ki_pitch"))  Ki_pitch  = doc["Ki_pitch"].as<float>();
      if (doc.containsKey("Kd_pitch"))  Kd_pitch  = doc["Kd_pitch"].as<float>();

      if (doc.containsKey("Kp_yaw"))    Kp_yaw    = doc["Kp_yaw"].as<float>();
      if (doc.containsKey("Ki_yaw"))    Ki_yaw    = doc["Ki_yaw"].as<float>();
      if (doc.containsKey("Kd_yaw"))    Kd_yaw    = doc["Kd_yaw"].as<float>();

      if (doc.containsKey("rollSetpoint"))   rollSetpoint   = doc["rollSetpoint"].as<float>();
      if (doc.containsKey("pitchSetpoint"))  pitchSetpoint  = doc["pitchSetpoint"].as<float>();
      if (doc.containsKey("yawSetpoint"))    yawSetpoint    = doc["yawSetpoint"].as<float>();

      Serial.println("[UDP] Updated PID/Setpoints from JSON.");
    } else {
      Serial.println("[UDP] JSON parse failed.");
    }
  }
}

void armAllESCs() {
  Serial.println("ARMING ESCs at Min Throttle...");
  writeMotorPWM(MOTOR_CH_1, 1000);
  writeMotorPWM(MOTOR_CH_2, 1000);
  writeMotorPWM(MOTOR_CH_3, 1000);
  writeMotorPWM(MOTOR_CH_4, 1000);
  delay(3000); // Wait 3s for ESC to beep/arm
  Serial.println("ESC Arming Complete.");
}

void writeMotorPWM(int channel, int pwmValue) {
  pwmValue = constrainESC(pwmValue);
  uint32_t dutyMax = (1 << PWM_RES_BITS) - 1;
  uint32_t dutyCycle = map(pwmValue, 
                           1000, 2000, 
                           (dutyMax*1000L)/20000L, 
                           (dutyMax*2000L)/20000L);

  ledcWrite(channel, dutyCycle);
}