#include <Wire.h>

// Gyroscope rates
float RateRoll, RatePitch, RateYaw;

// Calibration offsets
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

// Buffers pour stockage en RAM
const int maxSamples = 750; // 3 secondes à 4 ms = 750 échantillons
float bufferYaw[maxSamples];
float bufferTarget[maxSamples];
float bufferOutput[maxSamples];
unsigned long bufferTime[maxSamples];
int bufferIndex = 0;
bool plotActive = false;
bool plotDone = false;
unsigned long plotStartTime = 0;

// Accelerometer data
float AccX, AccY, AccZ;

// Angle estimates from accelerometer
float AngleRoll, AnglePitch;
float AngleYaw = 0;

// Kalman states
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float KalmanAngleYaw = 0, KalmanUncertaintyAngleYaw = 4;

float Kalman1DOutput[] = {0, 0};

// Loop timing
uint32_t LoopTimer;

// PID and motor control
const int motorLeftPin = 9;
const int motorRightPin = 10;
float yawTarget = 0;
float yawError = 0;
float yawPreviousError = 0;
float yawIntegral = 0;

float Kp = 1.0;
float Ki = 0;
float Kd = 0.05;
int baseSpeed = 130;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 16;

  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9);

  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  // Config accéléro
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  // Lecture accéléro
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Config gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Conversion
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / 3.142;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / 3.142;
}

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  pinMode(motorLeftPin, OUTPUT);
  pinMode(motorRightPin, OUTPUT);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibration
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  gyro_signals();
  RateYaw -= RateCalibrationYaw;
  AngleYaw += RateYaw * 0.004;
  kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, RateYaw, AngleYaw);
  KalmanAngleYaw = Kalman1DOutput[0];
  KalmanUncertaintyAngleYaw = Kalman1DOutput[1];
  yawTarget = KalmanAngleYaw;

  LoopTimer = micros();
}

void loop() {
  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  AngleYaw += RateYaw * 0.004;
  kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, RateYaw, AngleYaw);
  KalmanAngleYaw = Kalman1DOutput[0];
  KalmanUncertaintyAngleYaw = Kalman1DOutput[1];

  yawError = yawTarget - KalmanAngleYaw;
  yawIntegral += yawError * 0.004;
  float yawDerivative = (yawError - yawPreviousError) / 0.004;
  float yawOutput = Kp * yawError + Ki * yawIntegral + Kd * yawDerivative;
  yawPreviousError = yawError;

  yawOutput = constrain(yawOutput, -255, 255);

  int leftMotorSpeed = constrain(baseSpeed + yawOutput, 0, 255);
  int rightMotorSpeed = constrain(baseSpeed - yawOutput, 0, 255);

  analogWrite(motorLeftPin, leftMotorSpeed);
  analogWrite(motorRightPin, rightMotorSpeed);

  // Gestion des commandes
  static bool logging = false;
  static unsigned long logStartTime = 0;

  if (Serial.available()) {
    char command = Serial.read();
    if (command == '+' || command == '-') {
      if (command == '+') yawTarget += 50.0;
      if (command == '-') yawTarget -= 50.0;
      Serial.print("Nouvelle consigne Yaw: ");
      Serial.println(yawTarget);

      // Démarre le log
      logging = true;
      logStartTime = millis();
      Serial.println("Time(ms),Yaw,Target,Output"); // En-tête CSV
    }
  }

  // Log en temps réel (3 secondes)
  if (logging && (millis() - logStartTime <= 3000)) {
    if (micros() - LoopTimer >= 4000) {
      LoopTimer = micros();
      Serial.print(millis()); Serial.print(",");
      Serial.print(KalmanAngleYaw); Serial.print(",");
      Serial.print(yawTarget); Serial.print(",");
      Serial.println(yawOutput);
    }
  } else if (logging && millis() - logStartTime > 3000) {
    logging = false; // Stop logging
  }
}
