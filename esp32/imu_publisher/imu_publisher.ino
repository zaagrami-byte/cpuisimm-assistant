/**
 * ISIMM ROBOT — ESP32 : acquisition capteurs.
 * V1 : MPU6050 (I2C). Architecture prête pour ultrasons futurs.
 *
 * Protocole série USB (115200) :
 *   Boot : "READY"
 *   20 Hz : "I,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp>\n"
 *      a* en m/s^2 (float), g* en rad/s (float), temp en °C
 *   Futur : "U,<fl>,<fr>,<rl>,<rr>"  (distances ultrasons en mètres)
 *   Erreur init MPU : "E,MPU_FAIL" puis nouvelle tentative à 1 Hz.
 */

#include <Wire.h>

static const uint8_t MPU_ADDR      = 0x68;
static const int     SDA_PIN       = 21;
static const int     SCL_PIN       = 22;
static const float   ACC_SCALE     = 16384.0f;  // LSB par g (±2g)
static const float   GYRO_SCALE    = 131.0f;    // LSB par °/s (±250°/s)
static const float   DEG2RAD       = 0.017453292519943f;

static const unsigned long SEND_PERIOD_MS = 50;  // 20 Hz
unsigned long lastSend = 0;

bool initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);               // registre PWR_MGMT_1
  Wire.write(0x00);               // réveil
  if (Wire.endTransmission(true) != 0) return false;
  delay(100);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);               // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1, true);
  return (Wire.available() == 1) && (Wire.read() == 0x68);
}

void readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz, int16_t &temp) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);               // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 14, true);
  ax   = (int16_t)(Wire.read() << 8 | Wire.read());
  ay   = (int16_t)(Wire.read() << 8 | Wire.read());
  az   = (int16_t)(Wire.read() << 8 | Wire.read());
  temp = (int16_t)(Wire.read() << 8 | Wire.read());
  gx   = (int16_t)(Wire.read() << 8 | Wire.read());
  gy   = (int16_t)(Wire.read() << 8 | Wire.read());
  gz   = (int16_t)(Wire.read() << 8 | Wire.read());
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  if (initMPU()) {
    Serial.println("READY");
  } else {
    Serial.println("E,MPU_FAIL");
  }
  lastSend = millis();
}

void loop() {
  if (!initMPU_check()) { /* voir ci-dessous */ }

  unsigned long now = millis();
  if (now - lastSend < SEND_PERIOD_MS) return;
  lastSend = now;

  // Réessaie l'init MPU s'il a échoué (revient tout seul si câble/I2C revient)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 1, true);
  if (!(Wire.available() == 1 && Wire.read() == 0x68)) {
    Serial.println("E,MPU_FAIL");
    delay(1000);
    initMPU();
    return;
  }

  int16_t ax, ay, az, gx, gy, gz, temp;
  readMPU(ax, ay, az, gx, gy, gz, temp);

  float fax = ax / ACC_SCALE * 9.80665f;
  float fay = ay / ACC_SCALE * 9.80665f;
  float faz = az / ACC_SCALE * 9.80665f;
  float fgx = gx / GYRO_SCALE * DEG2RAD;
  float fgy = gy / GYRO_SCALE * DEG2RAD;
  float fgz = gz / GYRO_SCALE * DEG2RAD;
  float ftmp = temp / 340.0f + 36.53f;

  Serial.printf("I,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\n",
                fax, fay, faz, fgx, fgy, fgz, ftmp);

  // ------ EMPLACEMENT RÉSERVÉ ULTRASONS FUTURS ------
  // Lire 4 HC-SR04 puis : Serial.printf("U,%.2f,%.2f,%.2f,%.2f\n", fl, fr, rl, rr);
}
