/**
 * ISIMM ROBOT — ESP32 Sensor Node
 *
 * MPU6050 avec bibliothèque Adafruit.
 *
 * Connexion :
 *   MPU6050 VCC -> ESP32 3.3V
 *   MPU6050 GND -> ESP32 GND
 *   MPU6050 SDA -> ESP32 GPIO 21
 *   MPU6050 SCL -> ESP32 GPIO 22
 *
 * Protocole série USB : 115200 bauds
 *
 * Boot :
 *   READY
 *
 * IMU :
 *   I,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp>
 *
 * Unités :
 *   accélération : m/s²
 *   vitesse angulaire : rad/s
 *   température : °C
 *
 * Ultrasons futurs :
 *   U,<fl>,<fr>,<rl>,<rr>
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

static const unsigned long SEND_PERIOD_MS = 50;  // 20 Hz

Adafruit_MPU6050 mpu;

unsigned long lastSend = 0;
bool mpuOk = false;


/* =========================================================
 * INITIALISATION MPU6050
 * ========================================================= */

bool initMPU() {

  Serial.println("INIT_MPU");

  if (!mpu.begin()) {
    Serial.println("E,MPU_FAIL");
    return false;
  }

  /*
   * Accéléromètre ±2G
   *
   * Le robot est mobile mais ±2G suffit normalement
   * pour les accélérations classiques.
   */
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  /*
   * Gyroscope ±250 deg/s
   */
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  /*
   * Filtre passe-bas.
   * 21 Hz est adapté à une utilisation robotique
   * avec publication IMU à 20 Hz.
   */
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  Serial.println("MPU_OK");

  return true;
}


/* =========================================================
 * SETUP
 * ========================================================= */

void setup() {

  Serial.begin(115200);

  delay(500);

  /*
   * I2C ESP32
   */
  Wire.begin(SDA_PIN, SCL_PIN);

  /*
   * 100 kHz pour maximiser la robustesse.
   * On pourra passer à 400 kHz plus tard.
   */
  Wire.setClock(100000);

  /*
   * Initialisation MPU6050
   */
  mpuOk = initMPU();

  if (mpuOk) {
    Serial.println("READY");
  } else {
    Serial.println("E,MPU_FAIL");
  }

  lastSend = millis();
}


/* =========================================================
 * LOOP
 * ========================================================= */

void loop() {

  unsigned long now = millis();

  /*
   * Publication à 20 Hz
   */
  if (now - lastSend < SEND_PERIOD_MS) {
    return;
  }

  lastSend = now;


  /* ---------------------------------------------------------
   * Si le MPU n'a pas été initialisé, on réessaie.
   * --------------------------------------------------------- */

  if (!mpuOk) {

    mpuOk = initMPU();

    if (!mpuOk) {
      delay(1000);
      return;
    }

    Serial.println("READY");
  }


  /* ---------------------------------------------------------
   * Lecture MPU6050
   * --------------------------------------------------------- */

  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu.getEvent(
    &acceleration,
    &gyro,
    &temperature
  );


  /* ---------------------------------------------------------
   * Vérification basique
   * --------------------------------------------------------- */

  if (!isfinite(acceleration.acceleration.x) ||
      !isfinite(acceleration.acceleration.y) ||
      !isfinite(acceleration.acceleration.z) ||
      !isfinite(gyro.gyro.x) ||
      !isfinite(gyro.gyro.y) ||
      !isfinite(gyro.gyro.z)) {

    mpuOk = false;

    Serial.println("E,MPU_FAIL");

    return;
  }


  /* ---------------------------------------------------------
   * Envoi ROS 2 / Python
   *
   * Format EXACT :
   *
   * I,ax,ay,az,gx,gy,gz,temp
   * --------------------------------------------------------- */

  Serial.printf(
    "I,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\n",

    acceleration.acceleration.x,
    acceleration.acceleration.y,
    acceleration.acceleration.z,

    gyro.gyro.x,
    gyro.gyro.y,
    gyro.gyro.z,

    temperature.temperature
  );


  /* =========================================================
   * ULTRASONS FUTURS
   *
   * Lorsque les HC-SR04 seront ajoutés :
   *
   * float fl = ...;
   * float fr = ...;
   * float rl = ...;
   * float rr = ...;
   *
   * Serial.printf(
   *   "U,%.2f,%.2f,%.2f,%.2f\n",
   *   fl, fr, rl, rr
   * );
   *
   * ========================================================= */
}
