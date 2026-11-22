
#include <Wire.h>
#include <Servo.h>
#include <math.h>

const uint8_t MPU1 = 0x68;
const uint8_t MPU2 = 0x69;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo6;

const int PIN_M1 = 2;
const int PIN_M2 = 3;
const int PIN_M3 = 9;
const int PIN_M4 = 8;
const int PIN_M6 = 5;
const int BUTTON_PIN = 6;

const int GRIP_OPEN = 120;
const int GRIP_CLOSE = 30;
bool gripperOpen = false;
bool lastButtonState = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

struct SensorData {
  float angleY = 0;
  float angleZ = 0;
  float gyroBiasY = 0;
  float gyroBiasZ = 0;
  float initialY = 0;
  float initialZ = 0;
};

SensorData sensor1;
SensorData sensor2;

const float ACC_SCALE = 16384.0;
const float GYRO_SCALE = 131.0;

const float ALPHA = 0.98;

const float GAIN1 = 1.0;
const float GAIN2 = -1.0; 
const float GAIN3 = 1.0;
const float GAIN4 = 1.0;

unsigned long previousTime = 0;

bool writeRegister(
  uint8_t address,
  uint8_t reg,
  uint8_t value
) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission() == 0;
}

bool readMPU(
  uint8_t address,
  float &accX,
  float &accY,
  float &accZ,
  float &gyroY,
  float &gyroZ
) {
  Wire.beginTransmission(address);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t count = Wire.requestFrom(
    address, (uint8_t)14
  );

  if (count != 14) {
    return false;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read();

  Wire.read();
  Wire.read();

  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  accX = ax / ACC_SCALE;
  accY = ay / ACC_SCALE;
  accZ = az / ACC_SCALE;

  gyroY = gy / GYRO_SCALE;
  gyroZ = gz / GYRO_SCALE;

  return true;
}

bool calibrateSensor(
  uint8_t address,
  SensorData &data
) {
  const int N = 200;
  float sumY = 0;
  float sumZ = 0;

  float ax, ay, az, gy, gz;

  for (int i = 0; i < N; i++) {
    if (!readMPU(
      address, ax, ay, az, gy, gz
    )) {
      return false;
    }

    sumY += gy;
    sumZ += gz;

    delay(5);
  }

  data.gyroBiasY = sumY / N;
  data.gyroBiasZ = sumZ / N;

  data.angleY = atan2(
    -ax, sqrt(ay * ay + az * az)
  ) * 180.0 / PI;

  data.initialY = data.angleY;
  data.initialZ = 0;

  return true;
}

bool updateSensor(
  uint8_t address,
  SensorData &data,
  float dt
) {
  float ax, ay, az, gy, gz;

  if (!readMPU(
    address, ax, ay, az, gy, gz
  )) {
    return false;
  }

  gy -= data.gyroBiasY;
  gz -= data.gyroBiasZ;

  float accelAngleY = atan2(
    -ax, sqrt(ay * ay + az * az)
  ) * 180.0 / PI;

  data.angleY =
    ALPHA * (data.angleY + gy * dt)
    + (1.0 - ALPHA) * accelAngleY;

  data.angleZ += gz * dt;

  return true;
}

int calculateServoAngle(
  float relativeAngle,
  float gain
) {
  float target = 90 + relativeAngle * gain;

  return constrain(
    (int)round(target), 0, 180
  );
}

void updateGripper() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > 50) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        gripperOpen = !gripperOpen;

        servo6.write(
          gripperOpen ? GRIP_OPEN : GRIP_CLOSE
        );
      }
    }
  }

  lastButtonState = reading;
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(100000);

#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  servo1.attach(PIN_M1);
  servo2.attach(PIN_M2);
  servo3.attach(PIN_M3);
  servo4.attach(PIN_M4);
  servo6.attach(PIN_M6);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);

  
  servo6.write(GRIP_CLOSE);

  delay(1000);

  bool init1 = writeRegister(MPU1, 0x6B, 0x00);
  bool init2 = writeRegister(MPU2, 0x6B, 0x00);

  if (!init1 || !init2) {
    Serial.println("MPU initialization failed");
    while (true) {
      delay(1000);
    }
  }

  delay(100);

  Serial.println("Calibrating sensors...");

  bool cal1 = calibrateSensor(MPU1, sensor1);
  bool cal2 = calibrateSensor(MPU2, sensor2);

  if (!cal1 || !cal2) {
    Serial.println("Calibration failed");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Synchronization ready");

  previousTime = micros();
}

void loop() {
  unsigned long currentTime = micros();

  float dt =
    (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  
  if (dt <= 0 || dt > 0.1) {
    return;
  }

  
  bool valid1 = updateSensor(
    MPU1, sensor1, dt
  );

  bool valid2 = updateSensor(
    MPU2, sensor2, dt
  );

  
  if (valid1 && valid2) {

    float m1 = sensor1.angleZ
               - sensor1.initialZ;

    float m2 = sensor1.angleY
               - sensor1.initialY;

    float m3 = sensor2.angleY
               - sensor2.initialY;

    float m4 = sensor2.angleZ
               - sensor2.initialZ;

    
    servo1.write(
      calculateServoAngle(m1, GAIN1)
    );

    servo2.write(
      calculateServoAngle(m2, GAIN2)
    );

    servo3.write(
      calculateServoAngle(m3, GAIN3)
    );

    servo4.write(
      calculateServoAngle(m4, GAIN4)
    );
  }

  updateGripper();

  delay(10);
}
