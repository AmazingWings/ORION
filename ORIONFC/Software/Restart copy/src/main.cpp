#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_SPIFlash.h>
#include <Servo.h>

#define SEALEVELPRESSURE_HPA (1013.25)

#define GPS_RX_PIN  0
#define GPS_TX_PIN  1

#define TLM_SCK0    2
#define TLM_MOSI0   3
#define TLM_MISO0   4
#define TLM_CS0     5
#define TLM_G0      6
#define TLM_RST     7

#define SERVO1_PIN  8
#define SERVO2_PIN  9

#define FLASH_SCK1  10
#define FLASH_MOSI1 11
#define FLASH_MISO1 12
#define FLASH_CS1   13

#define BARO_SDA1   14
#define BARO_SCL1   15

#define IMU_CS1     17
#define BARO_INT1   18

#define IMU_SCK1    26
#define IMU_MOSI1   27
#define IMU_MISO1   28
#define IMU_INT1    29

#define SERVO1      SERVO1_PIN
#define SERVO_TRIM  0
#define SERVO_NEUTRAL (90 + SERVO_TRIM)

#define FLASH_SIZE          (8 * 1024 * 1024)
#define FLASH_METADATA_ADDR (FLASH_SIZE - 4096)
#define FLASH_LOG_MAX_ADDR  FLASH_METADATA_ADDR
#define METADATA_MAGIC      0xDEADBEEF

#define LAUNCH_ACCEL_THRESHOLD  5.0
#define LAUNCH_CONFIRM_COUNT    40
#define FLIGHT_DURATION         120000

#define MIN_ALTITUDE_AGL        20.0
#define MAX_TILT_ANGLE          45.0
#define VELOCITY_THRESHOLD      70.0
#define MAX_ROLL_RATE           300.0
#define MAX_ANGULAR_VEL         500.0
#define PRE_MANEUVER_MAX_RATE   60.0

#define KP_ROLL_ANGLE 3.6
#define KI_ROLL_ANGLE 0.03
#define KD_ROLL_ANGLE 0.14
#define KP_ROLL_RATE  1.7
#define KI_ROLL_RATE  0.24
#define KD_ROLL_RATE  0.08

#define ROLL_MANEUVER_TARGET    90.0
#define ROLL_MANEUVER_RAMP_TIME 1000
#define ROLL_MANEUVER_HOLD_TIME 2000

#define CONTROL_LOOP_HZ           200
#define CONTROL_LOOP_DT           (1.0 / CONTROL_LOOP_HZ)
#define CONTROL_LOOP_INTERVAL_US  (1000000 / CONTROL_LOOP_HZ)

#define COMP_FILTER_ALPHA 0.98

#define LANDING_ALT_AGL_MAX      30.0
#define LANDING_BARO_STABLE_THR  0.3
#define LANDING_CONFIRM_SAMPLES  200
#define LANDING_LOG_EXTRA_MS     10000

#define BURNOUT_ACCEL_THRESHOLD  2.0
#define BURNOUT_CONFIRM_COUNT    40

#define CAL_STDDEV_THRESHOLD     2.0
#define CAL_BIAS_WARN_THRESHOLD  3.0
#define CAL_DURATION_MS          8000

Adafruit_BMP3XX bmp;
Adafruit_LSM6DSO32 dso32;
Adafruit_FlashTransport_SPI flashTransport(FLASH_CS1, SPI1);
Adafruit_SPIFlash flash(&flashTransport);
Servo rollServo;

struct FlashMetadata {
  uint32_t magic;
  uint32_t logStartAddress;
  uint32_t logEndAddress;
  float gyro_bias_x;
  float gyro_bias_y;
  float gyro_bias_z;
  bool calibrationValid;
  uint8_t padding[3];
};

enum FlightState {
  STATE_MENU,
  STATE_IDLE,
  STATE_BOOST,
  STATE_COAST,
  STATE_ROLL_MANEUVER,
  STATE_DESCENT,
  STATE_LANDED,
  STATE_GROUND_TEST,
  STATE_ANGLE_TEST
};

FlightState currentState = STATE_MENU;
bool flightModeEnabled = false;
bool groundTestMode = false;
unsigned long groundTestStartTime = 0;
unsigned long angleTestStartTime = 0;
bool hasExceededVelocityThreshold = false;
bool rollDampingActive = false;
float padPressure_hPa = 0.0;
unsigned long coastStartTime = 0;
float lastBaroAlt = 0.0;
int stableAltCount = 0;
bool landingConfirmed = false;
unsigned long landingConfirmedTime = 0;
bool streamingData = false;
float roll_angle_gyro_global = 0;

struct FlightStats {
  float max_accel_boost;
  float max_accel_descent;
  float max_accel_total;
  float max_altitude;
  float max_velocity;
  float max_roll_rate;
  float max_angular_velocity;
  unsigned long burnout_time_us;
  unsigned long apogee_time_us;
  unsigned long landing_time_us;
  bool ejection_detected;
  unsigned long ejection_time_us;
  float ejection_accel_g;
} flightStats = {0};

struct CalibrationData {
  float gyro_bias_x;
  float gyro_bias_y;
  float gyro_bias_z;
  bool isCalibrated;
} calibration = {0, 0, 0, false};

struct FlightData {
  unsigned long timestamp_us;
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;
  float temperature;
  float pressure;
  float altitude_raw;
  float altitude_filtered;
  float velocity;
  float roll_angle;
  float roll_rate;
  float tilt_angle;
  float pitch_rate;
  float yaw_rate;
  float angular_velocity_magnitude;
  float total_accel_g;
  float servo_command;
  bool active_control_enabled;
  FlightState state;
  float gain_multiplier;
  float pid_error;
  float pid_integral;
  float pid_derivative;
};

FlightData currentData;

struct PIDController {
  float kp, ki, kd;
  float integral;
  float prev_error;
  float output_min, output_max;

  void init(float p, float i, float d, float out_min, float out_max) {
    kp = p; ki = i; kd = d;
    integral = 0; prev_error = 0;
    output_min = out_min; output_max = out_max;
  }

  float compute(float setpoint, float measurement, float dt, float gain_mult = 1.0) {
    float error = setpoint - measurement;
    float p_term = (kp * gain_mult) * error;
    integral += error * dt;
    integral = constrain(integral, -50.0, 50.0);
    float i_term = (ki * gain_mult) * integral;
    float derivative = (error - prev_error) / dt;
    float d_term = (kd * gain_mult) * derivative;
    prev_error = error;
    currentData.pid_error = error;
    currentData.pid_integral = integral;
    currentData.pid_derivative = derivative;
    return constrain(p_term + i_term + d_term, output_min, output_max);
  }

  void reset() {
    integral = 0;
    prev_error = 0;
  }
};

PIDController rollAnglePID;
PIDController rollRatePID;

struct KalmanFilter {
  float altitude;
  float velocity;
  float P[2][2];
  float Q[2][2];
  float R;

  void init() {
    altitude = 0; velocity = 0;
    P[0][0] = 1.0; P[0][1] = 0.0;
    P[1][0] = 0.0; P[1][1] = 1.0;
    Q[0][0] = 0.1; Q[0][1] = 0.0;
    Q[1][0] = 0.0; Q[1][1] = 1.0;
    R = 2.0;
  }

  void predict(float accel_z, float dt) {
    float net_accel = accel_z - 9.81;
    float new_altitude = altitude + velocity * dt + 0.5 * net_accel * dt * dt;
    float new_velocity = velocity + net_accel * dt;
    float F[2][2] = {{1.0, dt},{0.0, 1.0}};
    float P_temp[2][2];
    P_temp[0][0] = F[0][0]*P[0][0] + F[0][1]*P[1][0];
    P_temp[0][1] = F[0][0]*P[0][1] + F[0][1]*P[1][1];
    P_temp[1][0] = F[1][0]*P[0][0] + F[1][1]*P[1][0];
    P_temp[1][1] = F[1][0]*P[0][1] + F[1][1]*P[1][1];
    P[0][0] = P_temp[0][0] + Q[0][0];
    P[0][1] = P_temp[0][1];
    P[1][0] = P_temp[1][0];
    P[1][1] = P_temp[1][1] + Q[1][1];
    altitude = new_altitude;
    velocity = new_velocity;
  }

  void update(float altitude_measurement) {
    float y = altitude_measurement - altitude;
    float S = P[0][0] + R;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    altitude += K[0] * y;
    velocity += K[1] * y;
    float P_temp[2][2];
    P_temp[0][0] = (1.0 - K[0]) * P[0][0];
    P_temp[0][1] = (1.0 - K[0]) * P[0][1];
    P_temp[1][0] = P[1][0] - K[1] * P[0][0];
    P_temp[1][1] = P[1][1] - K[1] * P[0][1];
    memcpy(P, P_temp, sizeof(P));
  }
};

KalmanFilter altitudeKF;

char flashBuffer[4096];
uint16_t bufferIndex = 0;
uint32_t writeAddress = 0;
uint32_t logStartAddress = 0;
uint32_t logEndAddress = 0;
bool isLogging = false;

unsigned long launchTime_us = 0;
unsigned long lastControlUpdate_us = 0;
unsigned long flightStartTime = 0;
unsigned long rollManeuverStartTime = 0;

void initializeSensors();
void calibrateGyro();
void readSensors();
void updateStateEstimation();
void updateFlightState();
void updateFlightStats();
void runActiveControl();
void runGroundTest();
void runAngleTest();
void disableActiveControl();
void logDataToFlash();
void flushFlashBuffer();
void saveMetadataToFlash();
void loadMetadataFromFlash();
void displayMenu();
void handleMenuInput();
void sweepServos();
void enterFlightMode();
void enterGroundTestMode();
void enterAngleTestMode();
void readAllFlashData();
void eraseFlashData();
void flushSerialBuffer();
void printFlightSummary();
float calculateTiltAngle(float ax, float ay, float az);
float calculateGainMultiplier(float velocity);
void resetRollAngleTracking();
void handleLandingLogic();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("\n\n");
  Serial.println("╔═══════════════════════════════════════════════════════╗");
  Serial.println("║   ACTIVE ROLL CONTROL FLIGHT COMPUTER v3.0           ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.println();

  initializeSensors();
  loadMetadataFromFlash();

  rollAnglePID.init(KP_ROLL_ANGLE, KI_ROLL_ANGLE, KD_ROLL_ANGLE, -200.0, 200.0);
  rollRatePID.init(KP_ROLL_RATE, KI_ROLL_RATE, KD_ROLL_RATE, -80.0, 80.0);
  altitudeKF.init();

  rollServo.attach(SERVO1);
  rollServo.write(SERVO_NEUTRAL);

  Serial.println("\n✓ System Ready");
  if (calibration.isCalibrated) {
    Serial.println("✓ Calibration loaded from flash");
    Serial.print("  Gyro bias: [");
    Serial.print(calibration.gyro_bias_x, 2);
    Serial.print(", ");
    Serial.print(calibration.gyro_bias_y, 2);
    Serial.print(", ");
    Serial.print(calibration.gyro_bias_z, 2);
    Serial.println("] deg/s");
    Serial.println("  (Run 'c' to recalibrate if needed)");
  } else {
    Serial.println("⚠ No saved calibration found - please calibrate before flight");
  }

  Serial.println("\nPress 'M' or 'm' to open menu");
  Serial.println("═══════════════════════════════════════════════════════\n");

  currentState = STATE_MENU;
  delay(500);
}

void loop() {
  unsigned long currentTime_us = micros();

  if (currentState == STATE_MENU) {
    if (streamingData) {
      static unsigned long lastStreamPrint = 0;
      if (millis() - lastStreamPrint >= 100) {
        lastStreamPrint = millis();
        readSensors();
        updateStateEstimation();
        Serial.print("Roll: "); Serial.print(currentData.roll_angle, 1);
        Serial.print("°  Rate: "); Serial.print(currentData.roll_rate, 1);
        Serial.print("°/s  Alt: "); Serial.print(currentData.altitude_filtered, 1);
        Serial.print("m  Vel: "); Serial.print(currentData.velocity, 1);
        Serial.print("m/s  G: "); Serial.println(currentData.total_accel_g, 2);
      }
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 's' || c == 'S') {
          streamingData = false;
          Serial.println("\n✓ Stream stopped\n");
        }
      }
      return;
    }
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'M' || c == 'm') displayMenu();
    }
    handleMenuInput();
    return;
  }

  if (currentState == STATE_IDLE) {
    digitalWrite(LED_BUILTIN, HIGH);
    readSensors();
    float total_accel = sqrt(
      currentData.accel_x * currentData.accel_x +
      currentData.accel_y * currentData.accel_y +
      currentData.accel_z * currentData.accel_z
    ) / 9.81;

    static int launchConfirmCount = 0;
    if (total_accel > LAUNCH_ACCEL_THRESHOLD) {
      launchConfirmCount++;
      if (launchConfirmCount >= LAUNCH_CONFIRM_COUNT) {
        launchTime_us = currentTime_us;
        flightStartTime = millis();
        currentState = STATE_BOOST;
        isLogging = true;
        bufferIndex = 0;
        logStartAddress = writeAddress;
        String header = "Time_us,Gyro_X,Gyro_Y,Gyro_Z,Accel_X,Accel_Y,Accel_Z,Temp_C,Pressure_hPa,Alt_Raw_m,Alt_Filt_m,Velocity_ms,Roll_Angle_deg,Roll_Rate_degs,Tilt_deg,Pitch_Rate_degs,Yaw_Rate_degs,Angular_Vel_degs,Total_Accel_G,Servo_cmd_deg,Active_Ctrl,Gain_Mult,State,PID_Err,PID_Int,PID_Der\n";
        header.toCharArray(flashBuffer, header.length() + 1);
        bufferIndex = header.length();
        if (Serial) {
          Serial.println("\n!!! LAUNCH DETECTED - 5G SUSTAINED !!!");
          Serial.println("Starting data logging...\n");
        }
        launchConfirmCount = 0;
      }
    } else {
      launchConfirmCount = 0;
    }
    delay(1);
    return;
  }

  if (currentState == STATE_GROUND_TEST) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'X' || c == 'x') {
        if (Serial) Serial.println("\n⚠ GROUND TEST ABORTED BY USER\n");
        groundTestMode = false;
        disableActiveControl();
        currentState = STATE_MENU;
        return;
      }
    }
    if (currentTime_us - lastControlUpdate_us >= CONTROL_LOOP_INTERVAL_US) {
      lastControlUpdate_us = currentTime_us;
      readSensors();
      updateStateEstimation();
      runGroundTest();
      if ((millis() / 200) % 2 == 0) digitalWrite(LED_BUILTIN, HIGH);
      else digitalWrite(LED_BUILTIN, LOW);
    }
    return;
  }

  if (currentState == STATE_ANGLE_TEST) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'X' || c == 'x') {
        if (Serial) Serial.println("\n⚠ ANGLE TEST ABORTED BY USER\n");
        disableActiveControl();
        currentState = STATE_MENU;
        return;
      }
    }
    if (currentTime_us - lastControlUpdate_us >= CONTROL_LOOP_INTERVAL_US) {
      lastControlUpdate_us = currentTime_us;
      readSensors();
      updateStateEstimation();
      runAngleTest();
      if ((millis() / 100) % 2 == 0) digitalWrite(LED_BUILTIN, HIGH);
      else digitalWrite(LED_BUILTIN, LOW);
    }
    return;
  }

  if (currentTime_us - lastControlUpdate_us >= CONTROL_LOOP_INTERVAL_US) {
    lastControlUpdate_us = currentTime_us;
    readSensors();
    updateStateEstimation();
    updateFlightState();
    updateFlightStats();
    runActiveControl();
    logDataToFlash();
    handleLandingLogic();
    if ((millis() / 100) % 2 == 0) digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, LOW);
  }

  if (isLogging) {
    bool safety_timeout = (millis() - flightStartTime >= FLIGHT_DURATION);
    if (safety_timeout) {
      isLogging = false;
      flushFlashBuffer();
      saveMetadataToFlash();
      disableActiveControl();
      if (Serial) {
        Serial.println("\n═══════════════════════════════════════════════════════");
        Serial.println("  FLIGHT COMPLETE - SAFETY TIMEOUT (120s)");
        Serial.println("═══════════════════════════════════════════════════════");
        printFlightSummary();
        Serial.println("\nPress 'M' to open menu and retrieve data");
      }
      currentState = STATE_MENU;
    }
  }
}

void handleLandingLogic() {
  if (!isLogging) return;
  if (currentState == STATE_LANDED && !landingConfirmed) {
    landingConfirmed = true;
    landingConfirmedTime = millis();
    if (Serial) Serial.println("→ LANDING CONFIRMED - logging 10 more seconds...");
  }
  if (landingConfirmed && (millis() - landingConfirmedTime >= LANDING_LOG_EXTRA_MS)) {
    isLogging = false;
    flushFlashBuffer();
    saveMetadataToFlash();
    disableActiveControl();
    if (Serial) {
      Serial.println("\n═══════════════════════════════════════════════════════");
      Serial.println("  FLIGHT COMPLETE - LANDING DETECTED");
      Serial.print("  Flight time: ");
      Serial.print((millis() - flightStartTime) / 1000.0, 1);
      Serial.println(" seconds");
      Serial.println("═══════════════════════════════════════════════════════");
      printFlightSummary();
      Serial.println("\nPress 'M' to open menu and retrieve data");
    }
    currentState = STATE_MENU;
  }
}

void saveMetadataToFlash() {
  FlashMetadata meta;
  meta.magic = METADATA_MAGIC;
  meta.logStartAddress = logStartAddress;
  meta.logEndAddress = logEndAddress;
  meta.gyro_bias_x = calibration.gyro_bias_x;
  meta.gyro_bias_y = calibration.gyro_bias_y;
  meta.gyro_bias_z = calibration.gyro_bias_z;
  meta.calibrationValid = calibration.isCalibrated;
  flash.eraseSector(FLASH_METADATA_ADDR / 4096);
  flash.waitUntilReady();
  flash.writeBuffer(FLASH_METADATA_ADDR, (uint8_t*)&meta, sizeof(FlashMetadata));
  flash.waitUntilReady();
  if (Serial) Serial.println("✓ Metadata saved to flash");
}

void loadMetadataFromFlash() {
  FlashMetadata meta;
  flash.readBuffer(FLASH_METADATA_ADDR, (uint8_t*)&meta, sizeof(FlashMetadata));
  if (meta.magic != METADATA_MAGIC) {
    if (Serial) Serial.println("  No valid metadata found in flash (first boot or erased)");
    return;
  }
  logStartAddress = meta.logStartAddress;
  logEndAddress = meta.logEndAddress;
  writeAddress = logEndAddress;
  if (meta.calibrationValid) {
    calibration.gyro_bias_x = meta.gyro_bias_x;
    calibration.gyro_bias_y = meta.gyro_bias_y;
    calibration.gyro_bias_z = meta.gyro_bias_z;
    calibration.isCalibrated = true;
  }
}

void initializeSensors() {
  Serial.println("Initializing sensors...");

  Wire1.setSDA(BARO_SDA1);
  Wire1.setSCL(BARO_SCL1);
  Wire1.begin();

  if (!bmp.begin_I2C(0x77, &Wire1)) {
    if (!bmp.begin_I2C(0x76, &Wire1)) {
      Serial.println("✗ ERROR: BMP390 not found!");
    }
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
  Serial.println("✓ BMP390 initialized");

  SPI1.setRX(IMU_MISO1);
  SPI1.setTX(IMU_MOSI1);
  SPI1.setSCK(IMU_SCK1);
  SPI1.begin();

  if (!dso32.begin_SPI(IMU_CS1, &SPI1)) {
    Serial.println("✗ ERROR: LSM6DSO32 not found!");
  }
  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
  dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  dso32.setAccelDataRate(LSM6DS_RATE_416_HZ);
  dso32.setGyroDataRate(LSM6DS_RATE_416_HZ);
  Serial.println("✓ LSM6DSO32 initialized (±32G max)");

  pinMode(FLASH_CS1, OUTPUT);
  digitalWrite(FLASH_CS1, HIGH);
  SPI1.setRX(FLASH_MISO1);
  SPI1.setTX(FLASH_MOSI1);
  SPI1.setSCK(FLASH_SCK1);

  if (!flash.begin()) {
    Serial.println("✗ ERROR: Flash not found!");
  }
  uint8_t sanityBuf[4];
  flash.readBuffer(0, sanityBuf, 4);
  Serial.print("✓ Flash initialized (");
  Serial.print(flash.size() / 1024 / 1024);
  Serial.println(" MB)");
}

void calibrateGyro() {
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║           GYROSCOPE CALIBRATION                       ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.println("\nPlace rocket VERTICALLY (as it will be on the pad)");
  Serial.println("Keep COMPLETELY STILL during calibration");
  Serial.println("Press 'Y' when ready...");

  flushSerialBuffer();
  Serial.println("\nStabilizing... keep still!");
  delay(2000);

  Serial.print("Calibrating gyro bias (");
  Serial.print(CAL_DURATION_MS / 1000);
  Serial.println(" seconds)...");

  float sum_x = 0, sum_y = 0, sum_z = 0;
  float sum_sq_x = 0, sum_sq_y = 0, sum_sq_z = 0;
  float max_abs_z = 0;
  int samples = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < CAL_DURATION_MS) {
    sensors_event_t accel, gyro, temp;
    if (dso32.getEvent(&accel, &gyro, &temp)) {
      float gx = gyro.gyro.x * 57.2958;
      float gy = gyro.gyro.y * 57.2958;
      float gz = gyro.gyro.z * 57.2958;
      sum_x += gx; sum_y += gy; sum_z += gz;
      sum_sq_x += gx * gx;
      sum_sq_y += gy * gy;
      sum_sq_z += gz * gz;
      if (abs(gz) > max_abs_z) max_abs_z = abs(gz);
      samples++;
    }
    delay(5);
    if (samples % 40 == 0) Serial.print(".");
  }
  Serial.println();

  float bias_x = sum_x / samples;
  float bias_y = sum_y / samples;
  float bias_z = sum_z / samples;

  float stddev_z = sqrt((sum_sq_z / samples) - (bias_z * bias_z));
  float stddev_x = sqrt((sum_sq_x / samples) - (bias_x * bias_x));
  float stddev_y = sqrt((sum_sq_y / samples) - (bias_y * bias_y));

  bool calSuspect = false;
  if (stddev_x > CAL_STDDEV_THRESHOLD || stddev_y > CAL_STDDEV_THRESHOLD || stddev_z > CAL_STDDEV_THRESHOLD) {
    Serial.println("\n⚠ WARNING: High motion detected during calibration!");
    Serial.print("  StdDev: [");
    Serial.print(stddev_x, 2); Serial.print(", ");
    Serial.print(stddev_y, 2); Serial.print(", ");
    Serial.print(stddev_z, 2);
    Serial.println("] deg/s");
    Serial.println("  Rocket may have moved - consider recalibrating!");
    calSuspect = true;
  }

  if (abs(bias_z) > CAL_BIAS_WARN_THRESHOLD) {
    Serial.println("\n⚠ WARNING: Roll axis bias is unusually high!");
    Serial.print("  Roll bias: "); Serial.print(bias_z, 2); Serial.println(" deg/s");
    Serial.println("  This may cause roll angle drift - consider recalibrating");
    calSuspect = true;
  }

  calibration.gyro_bias_x = bias_x;
  calibration.gyro_bias_y = bias_y;
  calibration.gyro_bias_z = bias_z;
  calibration.isCalibrated = true;

  Serial.println("\n✓ Calibration complete!");
  Serial.print("  Gyro bias: [");
  Serial.print(bias_x, 2); Serial.print(", ");
  Serial.print(bias_y, 2); Serial.print(", ");
  Serial.print(bias_z, 2);
  Serial.println("] deg/s");

  if (calSuspect) Serial.println("  ⚠ Calibration saved but flagged as suspect - redo if possible");
  else Serial.println("  ✓ Calibration quality: GOOD");

  saveMetadataToFlash();
  Serial.println("  ✓ Calibration saved to flash\n");
}

void readSensors() {
  sensors_event_t accel, gyro, temp;
  dso32.getEvent(&accel, &gyro, &temp);

  float raw_gyro_x = gyro.gyro.x * 57.2958 - calibration.gyro_bias_x;
  float raw_gyro_y = gyro.gyro.y * 57.2958 - calibration.gyro_bias_y;
  float raw_gyro_z = gyro.gyro.z * 57.2958 - calibration.gyro_bias_z;

  currentData.gyro_x = raw_gyro_x;
  currentData.gyro_y = raw_gyro_z;
  currentData.gyro_z = raw_gyro_y;

  currentData.accel_x = accel.acceleration.x;
  currentData.accel_y = accel.acceleration.y;
  currentData.accel_z = accel.acceleration.z;

  currentData.gyro_x = constrain(currentData.gyro_x, -2000, 2000);
  currentData.gyro_y = constrain(currentData.gyro_y, -2000, 2000);
  currentData.gyro_z = constrain(currentData.gyro_z, -2000, 2000);

  currentData.accel_x = constrain(currentData.accel_x, -32*9.81, 32*9.81);
  currentData.accel_y = constrain(currentData.accel_y, -32*9.81, 32*9.81);
  currentData.accel_z = constrain(currentData.accel_z, -32*9.81, 32*9.81);

  currentData.pitch_rate = currentData.gyro_x;
  currentData.yaw_rate   = currentData.gyro_y;
  currentData.roll_rate  = currentData.gyro_z;

  currentData.angular_velocity_magnitude = sqrt(
    currentData.gyro_x * currentData.gyro_x +
    currentData.gyro_y * currentData.gyro_y +
    currentData.gyro_z * currentData.gyro_z
  );

  currentData.total_accel_g = sqrt(
    currentData.accel_x * currentData.accel_x +
    currentData.accel_y * currentData.accel_y +
    currentData.accel_z * currentData.accel_z
  ) / 9.81;

  if (bmp.performReading()) {
    currentData.temperature = bmp.temperature;
    currentData.pressure = bmp.pressure / 100.0;
    float currentPressure_hPa = bmp.pressure / 100.0;
    currentData.altitude_raw = 44330.0 * (1.0 - pow(currentPressure_hPa / padPressure_hPa, 0.1903));
    currentData.altitude_raw = constrain(currentData.altitude_raw, -10, 3000);
  }

  if (launchTime_us > 0) currentData.timestamp_us = micros() - launchTime_us;
  else currentData.timestamp_us = 0;
}

void updateStateEstimation() {
  altitudeKF.predict(currentData.accel_z, CONTROL_LOOP_DT);
  altitudeKF.update(currentData.altitude_raw);
  currentData.altitude_filtered = altitudeKF.altitude;
  currentData.velocity = altitudeKF.velocity;

  roll_angle_gyro_global += currentData.roll_rate * CONTROL_LOOP_DT;
  currentData.roll_angle = roll_angle_gyro_global;

  float ax = currentData.accel_x;
  float ay = currentData.accel_y;
  float az = currentData.accel_z;
  float total_accel = sqrt(ax*ax + ay*ay + az*az);
  if (abs(total_accel - 9.81) < 5.0) currentData.tilt_angle = calculateTiltAngle(ax, ay, az);
}

void updateFlightStats() {
  if (currentData.total_accel_g > flightStats.max_accel_total)
    flightStats.max_accel_total = currentData.total_accel_g;
  if (currentState == STATE_BOOST && currentData.total_accel_g > flightStats.max_accel_boost)
    flightStats.max_accel_boost = currentData.total_accel_g;
  if (currentState == STATE_DESCENT && currentData.total_accel_g > flightStats.max_accel_descent) {
    flightStats.max_accel_descent = currentData.total_accel_g;
    if (currentData.total_accel_g > 30.0 && !flightStats.ejection_detected) {
      flightStats.ejection_detected = true;
      flightStats.ejection_time_us = currentData.timestamp_us;
      flightStats.ejection_accel_g = currentData.total_accel_g;
      if (Serial) Serial.println("→ EJECTION CHARGE DETECTED!");
    }
  }
  if (currentData.altitude_filtered > flightStats.max_altitude)
    flightStats.max_altitude = currentData.altitude_filtered;
  if (currentData.velocity > flightStats.max_velocity)
    flightStats.max_velocity = currentData.velocity;
  if (abs(currentData.roll_rate) > flightStats.max_roll_rate)
    flightStats.max_roll_rate = abs(currentData.roll_rate);
  if (currentData.angular_velocity_magnitude > flightStats.max_angular_velocity)
    flightStats.max_angular_velocity = currentData.angular_velocity_magnitude;
}

void resetRollAngleTracking() {
  roll_angle_gyro_global = 0;
  currentData.roll_angle = 0;
  if (Serial) Serial.println("  Roll angle reset to 0°");
}

float calculateGainMultiplier(float velocity) {
  if (velocity < 20.0) return 1.0;
  float velocity_factor = velocity / VELOCITY_THRESHOLD;
  float multiplier = 1.0 / sqrt(velocity_factor);
  return constrain(multiplier, 0.5, 2.0);
}

void updateFlightState() {
  static int burnout_confirm = 0;

  switch (currentState) {
    case STATE_BOOST: {
      if (currentData.total_accel_g < BURNOUT_ACCEL_THRESHOLD) {
        burnout_confirm++;
        if (burnout_confirm >= BURNOUT_CONFIRM_COUNT) {
          currentState = STATE_COAST;
          coastStartTime = millis();
          flightStats.burnout_time_us = currentData.timestamp_us;
          resetRollAngleTracking();
          if (Serial) {
            Serial.println("→ STATE: COAST (Burnout detected)");
            Serial.println("→ ROLL DAMPING ENABLED");
          }
          burnout_confirm = 0;
        }
      } else {
        burnout_confirm = 0;
      }
      break;
    }

    case STATE_COAST: {
      if (currentData.velocity >= VELOCITY_THRESHOLD) {
        hasExceededVelocityThreshold = true;
        if (!rollDampingActive) {
          rollDampingActive = true;
          if (Serial) {
            Serial.print("→ ROLL DAMPING ACTIVE (v ≥ ");
            Serial.print(VELOCITY_THRESHOLD, 0);
            Serial.println(" m/s)");
          }
        }
      }

      if (hasExceededVelocityThreshold &&
          currentData.velocity < VELOCITY_THRESHOLD &&
          currentData.velocity > 0 &&
          abs(currentData.roll_rate) < PRE_MANEUVER_MAX_RATE) {
        rollAnglePID.reset();
        rollRatePID.reset();
        currentState = STATE_ROLL_MANEUVER;
        rollManeuverStartTime = millis();
        resetRollAngleTracking();
        if (Serial) {
          Serial.print("→ STATE: ROLL MANEUVER (v < ");
          Serial.print(VELOCITY_THRESHOLD, 0);
          Serial.println(" m/s)");
          Serial.println("  Executing: 0° → 90° → 0°");
        }
      }

      if (hasExceededVelocityThreshold &&
          currentData.velocity < VELOCITY_THRESHOLD &&
          currentData.velocity > 0 &&
          abs(currentData.roll_rate) >= PRE_MANEUVER_MAX_RATE) {
        if (Serial) Serial.println("⚠ Maneuver skipped - spin rate too high, continuing damping");
      }

      if (currentData.velocity <= 1.0 && currentData.altitude_filtered > 50.0) {
        currentState = STATE_DESCENT;
        flightStats.apogee_time_us = currentData.timestamp_us;
        disableActiveControl();
        if (Serial) Serial.println("→ STATE: DESCENT (Apogee reached)");
      }
      break;
    }

    case STATE_ROLL_MANEUVER: {
      unsigned long totalManeuverTime = 2 * ROLL_MANEUVER_RAMP_TIME + ROLL_MANEUVER_HOLD_TIME;
      if (millis() - rollManeuverStartTime >= totalManeuverTime) {
        currentState = STATE_COAST;
        if (Serial) {
          Serial.println("→ STATE: COAST (Maneuver complete)");
          Serial.println("→ ROLL DAMPING RESUMED");
        }
      }
      if (currentData.tilt_angle > MAX_TILT_ANGLE) {
        currentState = STATE_COAST;
        if (Serial) Serial.println("⚠ MANEUVER ABORTED: Excessive tilt");
      }
      if (abs(currentData.roll_rate) > MAX_ROLL_RATE) {
        currentState = STATE_COAST;
        if (Serial) Serial.println("⚠ MANEUVER ABORTED: Excessive roll rate");
      }
      if (currentData.velocity <= 1.0 && currentData.altitude_filtered > 50.0) {
        currentState = STATE_DESCENT;
        flightStats.apogee_time_us = currentData.timestamp_us;
        disableActiveControl();
        if (Serial) Serial.println("→ STATE: DESCENT (Apogee during maneuver)");
      }
      break;
    }

    case STATE_DESCENT: {
      float alt_change = abs(currentData.altitude_raw - lastBaroAlt);
      lastBaroAlt = currentData.altitude_raw;
      if (alt_change < LANDING_BARO_STABLE_THR && currentData.altitude_filtered < LANDING_ALT_AGL_MAX) {
        stableAltCount++;
        if (stableAltCount >= LANDING_CONFIRM_SAMPLES) {
          currentState = STATE_LANDED;
          flightStats.landing_time_us = currentData.timestamp_us;
          if (Serial) Serial.println("→ STATE: LANDED (Baro stable near ground)");
          stableAltCount = 0;
        }
      } else {
        stableAltCount = 0;
      }
      break;
    }

    default: break;
  }

  currentData.state = currentState;
}

void runActiveControl() {
  if (currentState == STATE_DESCENT || currentState == STATE_LANDED) {
    rollServo.write(SERVO_NEUTRAL);
    currentData.active_control_enabled = false;
    currentData.gain_multiplier = 1.0;
    return;
  }

  bool shouldControl = false;
  float angle_setpoint = 0.0;
  float rate_setpoint = 0.0;
  bool useAngleControl = false;

  if (currentState == STATE_BOOST) {
    rollServo.write(SERVO_NEUTRAL);
    currentData.servo_command = SERVO_NEUTRAL;
    currentData.active_control_enabled = false;
    currentData.gain_multiplier = 1.0;
    return;
  } else if (currentData.altitude_filtered < MIN_ALTITUDE_AGL) {
    shouldControl = false;
  } else if (currentData.tilt_angle > MAX_TILT_ANGLE) {
    shouldControl = false;
    if (Serial) Serial.println("⚠ CONTROL ABORT: Tilt > 45°");
  } else if (currentData.angular_velocity_magnitude > MAX_ANGULAR_VEL) {
    shouldControl = false;
    if (Serial) Serial.println("⚠ CONTROL ABORT: Tumbling detected");
  } else if (currentState == STATE_ROLL_MANEUVER) {
    shouldControl = true;
    useAngleControl = true;
    unsigned long elapsed = millis() - rollManeuverStartTime;
    if (elapsed < ROLL_MANEUVER_RAMP_TIME) {
      angle_setpoint = ROLL_MANEUVER_TARGET * (float(elapsed) / ROLL_MANEUVER_RAMP_TIME);
    } else if (elapsed < ROLL_MANEUVER_RAMP_TIME + ROLL_MANEUVER_HOLD_TIME) {
      angle_setpoint = ROLL_MANEUVER_TARGET;
    } else {
      unsigned long rampDownElapsed = elapsed - ROLL_MANEUVER_RAMP_TIME - ROLL_MANEUVER_HOLD_TIME;
      angle_setpoint = ROLL_MANEUVER_TARGET * (1.0 - float(rampDownElapsed) / ROLL_MANEUVER_RAMP_TIME);
      angle_setpoint = max(0.0f, angle_setpoint);
    }
  } else if (currentState == STATE_COAST) {
    shouldControl = true;
    useAngleControl = false;
    rate_setpoint = 0.0;
  }

  if (shouldControl) {
    currentData.gain_multiplier = calculateGainMultiplier(currentData.velocity);
    float control_output;
    if (useAngleControl) {
      float desired_rate = rollAnglePID.compute(angle_setpoint, currentData.roll_angle, CONTROL_LOOP_DT, currentData.gain_multiplier);
      control_output = rollRatePID.compute(desired_rate, currentData.roll_rate, CONTROL_LOOP_DT, currentData.gain_multiplier);
    } else {
      control_output = rollRatePID.compute(rate_setpoint, currentData.roll_rate, CONTROL_LOOP_DT, currentData.gain_multiplier);
    }
    currentData.servo_command = SERVO_NEUTRAL + control_output;
    currentData.servo_command = constrain(currentData.servo_command, 0, 180);

    static float last_servo_position = SERVO_NEUTRAL;
    float max_servo_rate = 300.0;
    float max_step = max_servo_rate * CONTROL_LOOP_DT;
    float commanded_change = currentData.servo_command - last_servo_position;
    commanded_change = constrain(commanded_change, -max_step, max_step);
    currentData.servo_command = last_servo_position + commanded_change;
    last_servo_position = currentData.servo_command;

    rollServo.write((int)round(currentData.servo_command));
    currentData.active_control_enabled = true;
  } else {
    disableActiveControl();
    currentData.gain_multiplier = 1.0;
  }
}

void runGroundTest() {
  unsigned long elapsed = millis() - groundTestStartTime;
  if (elapsed >= 30000) {
    if (Serial) {
      Serial.println("\n╔═══════════════════════════════════════════════════════╗");
      Serial.println("║         GROUND TEST COMPLETE                          ║");
      Serial.println("╚═══════════════════════════════════════════════════════╝");
      Serial.println("\n✓ Rate damping tested for 30 seconds");
      Serial.println("\nVERIFY SERVO DIRECTION:");
      Serial.println("  • Did servos RESIST your manual rotation?");
      Serial.println("  • When stationary, did servos return to neutral?");
      Serial.println("\n⚠ IF SERVOS HELPED YOU SPIN → ABORT FLIGHT!");
      Serial.println("  → Flip gyro_z sign in readSensors()\n");
    }
    groundTestMode = false;
    disableActiveControl();
    currentState = STATE_MENU;
    return;
  }
  float control_output = rollRatePID.compute(0.0, currentData.roll_rate, CONTROL_LOOP_DT, 1.0);
  currentData.servo_command = SERVO_NEUTRAL + control_output;
  currentData.servo_command = constrain(currentData.servo_command, 0, 180);
  rollServo.write((int)round(currentData.servo_command));
  currentData.active_control_enabled = true;
  if (Serial && elapsed % 1000 < 5) {
    Serial.print("  ["); Serial.print(elapsed / 1000);
    Serial.print("s]  Roll: "); Serial.print(currentData.roll_angle, 1);
    Serial.print("°  Rate: "); Serial.print(currentData.roll_rate, 1);
    Serial.print("°/s  Servo: "); Serial.print(currentData.servo_command, 1);
    Serial.println("°");
  }
}

void runAngleTest() {
  unsigned long elapsed = millis() - angleTestStartTime;
  unsigned long totalManeuverTime = 2 * ROLL_MANEUVER_RAMP_TIME + ROLL_MANEUVER_HOLD_TIME;
  if (elapsed >= totalManeuverTime + 1000) {
    if (Serial) {
      Serial.println("\n╔═══════════════════════════════════════════════════════╗");
      Serial.println("║         ANGLE TEST COMPLETE                           ║");
      Serial.println("╚═══════════════════════════════════════════════════════╝");
      Serial.println("\n✓ 90° maneuver tested (0° → 90° → 0°)");
      Serial.println("\nREVIEW SERVO MOVEMENTS:");
      Serial.println("  • Smooth ramp to 90°?");
      Serial.println("  • Steady hold for 2 seconds?");
      Serial.println("  • Smooth return to 0°?");
      Serial.println("  • No oscillation or overshoot?\n");
    }
    disableActiveControl();
    currentState = STATE_MENU;
    return;
  }
  float angle_setpoint;
  if (elapsed < ROLL_MANEUVER_RAMP_TIME) {
    angle_setpoint = ROLL_MANEUVER_TARGET * (float(elapsed) / ROLL_MANEUVER_RAMP_TIME);
  } else if (elapsed < ROLL_MANEUVER_RAMP_TIME + ROLL_MANEUVER_HOLD_TIME) {
    angle_setpoint = ROLL_MANEUVER_TARGET;
  } else {
    unsigned long rampDownElapsed = elapsed - ROLL_MANEUVER_RAMP_TIME - ROLL_MANEUVER_HOLD_TIME;
    angle_setpoint = ROLL_MANEUVER_TARGET * (1.0 - float(rampDownElapsed) / ROLL_MANEUVER_RAMP_TIME);
    angle_setpoint = max(0.0f, angle_setpoint);
  }
  float desired_rate = rollAnglePID.compute(angle_setpoint, currentData.roll_angle, CONTROL_LOOP_DT, 1.0);
  float control_output = rollRatePID.compute(desired_rate, currentData.roll_rate, CONTROL_LOOP_DT, 1.0);
  currentData.servo_command = SERVO_NEUTRAL + control_output;
  currentData.servo_command = constrain(currentData.servo_command, 0, 180);
  rollServo.write((int)round(currentData.servo_command));
  currentData.active_control_enabled = true;
  if (Serial && elapsed % 200 < 5) {
    Serial.print("  ["); Serial.print(elapsed / 1000.0, 1);
    Serial.print("s]  Target: "); Serial.print(angle_setpoint, 1);
    Serial.print("°  Actual: "); Serial.print(currentData.roll_angle, 1);
    Serial.print("°  Error: "); Serial.print(angle_setpoint - currentData.roll_angle, 1);
    Serial.println("°");
  }
}

void disableActiveControl() {
  rollServo.write(SERVO_NEUTRAL);
  rollAnglePID.reset();
  rollRatePID.reset();
  currentData.servo_command = SERVO_NEUTRAL;
  currentData.active_control_enabled = false;
}

void logDataToFlash() {
  if (!isLogging) return;
  if (writeAddress + 512 >= FLASH_LOG_MAX_ADDR) {
    isLogging = false;
    flushFlashBuffer();
    saveMetadataToFlash();
    if (Serial) Serial.println("⚠ FLASH FULL - logging stopped");
    return;
  }
  char line[400];
  snprintf(line, sizeof(line),
    "%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%.2f,%d,%.2f,%.2f,%.2f\n",
    currentData.timestamp_us,
    currentData.gyro_x, currentData.gyro_y, currentData.gyro_z,
    currentData.accel_x, currentData.accel_y, currentData.accel_z,
    currentData.temperature, currentData.pressure,
    currentData.altitude_raw, currentData.altitude_filtered, currentData.velocity,
    currentData.roll_angle, currentData.roll_rate, currentData.tilt_angle,
    currentData.pitch_rate, currentData.yaw_rate,
    currentData.angular_velocity_magnitude, currentData.total_accel_g,
    currentData.servo_command,
    currentData.active_control_enabled ? 1 : 0,
    currentData.gain_multiplier,
    (int)currentData.state,
    currentData.pid_error, currentData.pid_integral, currentData.pid_derivative
  );
  int lineLen = strlen(line);
  if (bufferIndex + lineLen >= sizeof(flashBuffer)) {
    bool success = flash.writeBuffer(writeAddress, (uint8_t*)flashBuffer, bufferIndex);
    if (!success && Serial) Serial.println("⚠ FLASH WRITE FAILED!");
    flash.waitUntilReady();
    writeAddress += bufferIndex;
    bufferIndex = 0;
  }
  strcpy(flashBuffer + bufferIndex, line);
  bufferIndex += lineLen;
}

void flushFlashBuffer() {
  if (bufferIndex > 0) {
    flash.writeBuffer(writeAddress, (uint8_t*)flashBuffer, bufferIndex);
    flash.waitUntilReady();
    writeAddress += bufferIndex;
    logEndAddress = writeAddress;
    bufferIndex = 0;
  }
}

void printFlightSummary() {
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║              FLIGHT SUMMARY                           ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝\n");
  Serial.print("Max Altitude:        "); Serial.print(flightStats.max_altitude, 1); Serial.println(" m AGL");
  Serial.print("Max Velocity:        "); Serial.print(flightStats.max_velocity, 1); Serial.println(" m/s");
  Serial.print("Burnout Time:        "); Serial.print(flightStats.burnout_time_us / 1000000.0, 2); Serial.println(" s");
  Serial.print("Apogee Time:         "); Serial.print(flightStats.apogee_time_us / 1000000.0, 2); Serial.println(" s");
  Serial.println();
  Serial.print("Max Accel (Boost):   "); Serial.print(flightStats.max_accel_boost, 1); Serial.println(" Gs");
  Serial.print("Max Accel (Descent): "); Serial.print(flightStats.max_accel_descent, 1); Serial.println(" Gs");
  if (flightStats.ejection_detected) {
    Serial.print("  └─ Ejection at:    ");
    Serial.print(flightStats.ejection_time_us / 1000000.0, 2);
    Serial.print(" s ("); Serial.print(flightStats.ejection_accel_g, 1); Serial.println(" Gs)");
  }
  Serial.println();
  Serial.print("Max Roll Rate:       "); Serial.print(flightStats.max_roll_rate, 1); Serial.println(" deg/s");
  Serial.print("Max Angular Vel:     "); Serial.print(flightStats.max_angular_velocity, 1); Serial.println(" deg/s\n");
}

void flushSerialBuffer() {
  while (Serial.available()) Serial.read();
}

void displayMenu() {
  flushSerialBuffer();
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║            FLIGHT CONTROL MENU                        ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("  r - Read all data from flash");
  Serial.println("  e - Erase all flash data");
  Serial.println("  s - Sweep servos (0-180°)");
  Serial.println("  c - Calibrate gyroscope");
  Serial.println("  f - Enter FLIGHT MODE (arm system)");
  Serial.println("  g - Ground test mode (rate damping test)");
  Serial.println("  a - Angle test mode (90° maneuver test)");
  Serial.println("  d - Live data stream");
  Serial.println("  m - Show this menu");
  Serial.println();
  Serial.print("→ ");
}

void handleMenuInput() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  flushSerialBuffer();
  Serial.println(cmd);
  switch (cmd) {
    case 'r': case 'R': readAllFlashData(); break;
    case 'e': case 'E': eraseFlashData(); break;
    case 's': case 'S': sweepServos(); break;
    case 'c': case 'C': calibrateGyro(); break;
    case 'f': case 'F': enterFlightMode(); break;
    case 'g': case 'G': enterGroundTestMode(); break;
    case 'a': case 'A': enterAngleTestMode(); break;
    case 'd': case 'D':
      streamingData = true;
      Serial.println("\n╔═══════════════════════════════════════════════════════╗");
      Serial.println("║            LIVE DATA STREAM                           ║");
      Serial.println("╚═══════════════════════════════════════════════════════╝");
      Serial.println("Press 's' to stop\n");
      break;
    case 'm': case 'M': displayMenu(); break;
    default: break;
  }
}

void sweepServos() {
  Serial.println("\nSweeping servos 0° → 180° → 0°");
  for (int angle = 0; angle <= 180; angle += 5) { rollServo.write(angle); Serial.print("."); delay(20); }
  Serial.println();
  for (int angle = 180; angle >= 0; angle -= 5) { rollServo.write(angle); Serial.print("."); delay(20); }
  Serial.println();
  rollServo.write(SERVO_NEUTRAL);
  Serial.println("✓ Sweep complete - returned to neutral\n");
}

void enterFlightMode() {
  if (!calibration.isCalibrated) {
    Serial.println("\n✗ ERROR: Gyroscope not calibrated!");
    Serial.println("Run calibration first (press 'c')\n");
    return;
  }
  Serial.println("\n⚠  You are about to ARM the flight computer.");
  Serial.println("Type 'ARM' and press Enter to confirm:");
  flushSerialBuffer();
  String input = "";
  unsigned long startWait = millis();
  while (millis() - startWait < 15000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        input.trim();
        if (input == "ARM") break;
        else {
          Serial.println("✗ Arming cancelled.\n");
          return;
        }
      } else {
        input += c;
        Serial.print(c);
      }
    }
  }
  if (input != "ARM") {
    Serial.println("\n✗ Arming timed out.\n");
    return;
  }
  delay(100);
  if (bmp.performReading()) {
    padPressure_hPa = bmp.pressure / 100.0;
    Serial.print("\n✓ Pad pressure updated: ");
    Serial.print(padPressure_hPa, 2);
    Serial.println(" hPa");
  }
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║              SYSTEM ARMED                             ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("⚠  ARMED - READY FOR LAUNCH");
  Serial.println();
  Serial.println("  • Waiting for 5G sustained acceleration");
  Serial.println("  • NO CONTROL during boost");
  Serial.println("  • Roll damping after burnout");
  Serial.println("  • 90° roll maneuver when v < 70 m/s");
  Serial.println("  • Control disabled at apogee");
  Serial.println("  • Auto-stop logging 10s after landing");
  Serial.println();
  Serial.println("LED SOLID = armed, LED BLINKING = flight active");
  Serial.println("You may now unplug and place on pad.");
  Serial.println("═══════════════════════════════════════════════════════");
  currentState = STATE_IDLE;
  flightModeEnabled = true;
  memset(&flightStats, 0, sizeof(flightStats));
  altitudeKF.init();
  rollAnglePID.reset();
  rollRatePID.reset();
  resetRollAngleTracking();
  launchTime_us = 0;
  hasExceededVelocityThreshold = false;
  rollDampingActive = false;
  landingConfirmed = false;
  stableAltCount = 0;
  lastBaroAlt = 0;
}

void enterGroundTestMode() {
  if (!calibration.isCalibrated) {
    Serial.println("\n✗ ERROR: Not calibrated! Run 'c' first.\n");
    return;
  }
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║           GROUND TEST MODE - RATE DAMPING             ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.println("\n⚠  ACTIVE CONTROL ENABLED - Test duration: 30 seconds");
  Serial.println("\nINSTRUCTIONS:");
  Serial.println("  1. Manually ROLL the rocket CLOCKWISE");
  Serial.println("  2. Servos should push COUNTER-CLOCKWISE (resisting)");
  Serial.println("  3. When you stop, servos should return to neutral");
  Serial.println("\n⚠  IF SERVOS HELP YOU SPIN → ABORT FLIGHT!");
  Serial.println("\nPress 'X' to abort. Starting in 3...");
  delay(1000); Serial.println("2...");
  delay(1000); Serial.println("1...");
  delay(1000);
  Serial.println("\n✓ GROUND TEST STARTED\n");
  currentState = STATE_GROUND_TEST;
  groundTestMode = true;
  groundTestStartTime = millis();
  altitudeKF.init();
  rollAnglePID.reset();
  rollRatePID.reset();
  resetRollAngleTracking();
  lastControlUpdate_us = micros();
}

void enterAngleTestMode() {
  if (!calibration.isCalibrated) {
    Serial.println("\n✗ ERROR: Not calibrated! Run 'c' first.\n");
    return;
  }
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║           ANGLE TEST MODE - 90° MANEUVER              ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.println("\n  0° → 90° (1s ramp) → Hold 90° (2s) → 90° → 0° (1s ramp)");
  Serial.println("\nPress 'X' to abort. Starting in 3...");
  delay(1000); Serial.println("2...");
  delay(1000); Serial.println("1...");
  delay(1000);
  Serial.println("\n✓ ANGLE TEST STARTED\n");
  currentState = STATE_ANGLE_TEST;
  angleTestStartTime = millis();
  altitudeKF.init();
  rollAnglePID.reset();
  rollRatePID.reset();
  resetRollAngleTracking();
  lastControlUpdate_us = micros();
}

void readAllFlashData() {
  if (logEndAddress == 0 || logEndAddress <= logStartAddress) {
    Serial.println("\n✗ No flight data found in flash\n");
    Serial.println("  (If you just landed, metadata saves at end of flight)");
    return;
  }
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║              FLIGHT DATA CSV EXPORT                   ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.print("\nReading "); Serial.print((logEndAddress - logStartAddress) / 1024);
  Serial.println(" KB from flash...");
  Serial.println("\n========== CSV DATA START ==========\n");
  uint32_t address = logStartAddress;
  uint8_t buffer[256];
  while (address < logEndAddress) {
    uint32_t bytesToRead = min((uint32_t)256, logEndAddress - address);
    flash.readBuffer(address, buffer, bytesToRead);
    flash.waitUntilReady();
    for (uint32_t i = 0; i < bytesToRead; i++) Serial.write(buffer[i]);
    address += bytesToRead;
  }
  Serial.println("\n========== CSV DATA END ==========");
  Serial.println("\nCopy the above data and save as .csv file\n");
}

void eraseFlashData() {
  Serial.println("\n  ⚠  WARNING: This will erase ALL flight data!");
  Serial.println("  Type 'Y' to confirm (10 second timeout)...");
  flushSerialBuffer();
  unsigned long startWait = millis();
  while (millis() - startWait < 10000) {
    if (Serial.available()) {
      char c = Serial.read();
      flushSerialBuffer();
      if (c == 'Y' || c == 'y') {
        Serial.println("\nErasing flash memory...");
        uint32_t sectors = (writeAddress / 4096) + 10;
        for (uint32_t i = 0; i < sectors; i++) {
          flash.eraseSector(i);
          flash.waitUntilReady();
          if (i % 10 == 0) Serial.print(".");
        }
        writeAddress = 0;
        logStartAddress = 0;
        logEndAddress = 0;
        bufferIndex = 0;
        flash.eraseSector(FLASH_METADATA_ADDR / 4096);
        flash.waitUntilReady();
        Serial.println("\n✓ Flash memory erased (including metadata)\n");
        return;
      } else {
        Serial.println("\n✗ Erase cancelled\n");
        return;
      }
    }
  }
  Serial.println("\n✗ Erase timed out - cancelled\n");
}

float calculateTiltAngle(float ax, float ay, float az) {
  float magnitude = sqrt(ax*ax + ay*ay + az*az);
  if (magnitude < 0.1) return 0;
  float cosAngle = constrain(az / magnitude, -1.0, 1.0);
  return acos(cosAngle) * 57.2958;
}
