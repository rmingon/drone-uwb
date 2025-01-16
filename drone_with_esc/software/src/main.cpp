#include <Wire.h>
#include "MPU6050.h"

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
float Kp_roll  = 2.0f, Ki_roll  = 0.0f, Kd_roll  = 1.0f;
float Kp_pitch = 2.0f, Ki_pitch = 0.0f, Kd_pitch = 1.0f;
float Kp_yaw   = 2.0f, Ki_yaw   = 0.0f, Kd_yaw   = 1.0f;

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
const int motorPin1 = 25;   // front-right
const int motorPin2 = 26;   // front-left
const int motorPin3 = 27;   // rear-left
const int motorPin4 = 32;   // rear-right

int baseThrottle = 1200;  // Adjust for your motors/ESC

int constrainESC(int pwmValue) {
  if (pwmValue < 1000) return 1000;
  if (pwmValue > 2000) return 2000;
  return pwmValue;
}

void writeMotorPWM(int motorPin, int pwmValue) {
  int dutyCycle = map(pwmValue, 1000, 2000, 0, 255);
  analogWrite(motorPin, dutyCycle);
}

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

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  initKalman(kalmanRoll,  0.001f, 0.003f, 0.03f);
  initKalman(kalmanPitch, 0.001f, 0.003f, 0.03f);
  initKalman(kalmanYaw,   0.001f, 0.003f, 0.03f);

  previousTime = micros();

  // Arm ESCs by sending minimum throttle
  writeMotorPWM(motorPin1, 1000);
  writeMotorPWM(motorPin2, 1000);
  writeMotorPWM(motorPin3, 1000);
  writeMotorPWM(motorPin4, 1000);
  delay(3000); // Wait for ESCs to arm
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

  // 4. PID
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

  writeMotorPWM(motorPin1, motor1);
  writeMotorPWM(motorPin2, motor2);
  writeMotorPWM(motorPin3, motor3);
  writeMotorPWM(motorPin4, motor4);

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