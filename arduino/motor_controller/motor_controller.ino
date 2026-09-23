/**
 * ISIMM ROBOT — Arduino UNO : contrôle bas niveau des 2 moteurs hoverboard.
 *
 * AUCUN encodeur. Cette carte ne fait QUE :
 *   - recevoir des commandes série de la Raspberry Pi ;
 *   - générer PWM / DIR / BRAKE ;
 *   - surveiller la communication (watchdog) ;
 *   - gérer l'arrêt d'urgence (logiciel + broche optionnelle pour bouton physique).
 *
 * Elle ne connaît NI cartes, NI obstacles, NI Nav2, NI destinations.
 *
 * PROTOCOLE (baud 115200, '\n' en fin de ligne) :
 *   M,<lpwm>,<ldir>,<lbrk>,<rpwm>,<rdir>,<rbrk>   commande moteurs
 *        pwm  : 0..255   dir : 0/1   brk : 0/1
 *   S   -> emergency stop (verrouillé)
 *   C   -> clear emergency stop
 *   P   -> ping  (répond "OK")
 * Boot : envoie "READY". Toute commande invalide -> moteurs STOP + "ERR".
 * Watchdog : si aucune commande M valide pendant CMD_TIMEOUT_MS -> BRAKE.
 */

// ---------- Réglages matériels (À ADAPTER si le driver est actif bas) ----------
const uint8_t PIN_R_BRAKE = 8;
const uint8_t PIN_R_DIR   = 9;
const uint8_t PIN_R_PWM   = 10;
const uint8_t PIN_L_BRAKE = 12;
const uint8_t PIN_L_DIR   = 13;
const uint8_t PIN_L_PWM   = 11;

const uint8_t BRAKE_ACTIVE_LEVEL   = HIGH;  // état de la broche BRAKE qui freine
const uint8_t DIR_FORWARD_LEVEL    = HIGH;  // état de la broche DIR qui fait avancer

const uint8_t PIN_ESTOP_BUTTON = 2;         // bouton d'urgence physique OPTIONNEL (vers GND)
const uint8_t PIN_STATUS_LED   = 4;         // LED de statut (allumée = en sécurité/arrêt)

const unsigned long CMD_TIMEOUT_MS = 500;   // watchdog communication (doit être > période d'envoi du Pi)
const unsigned long WATCHDOG_CHECK_MS = 10; // période de surveillance
// -----------------------------------------------------------------------------

volatile bool estop = false;                // arrêt d'urgence (ISR + commande S)
bool          serialEstop = false;

unsigned long lastCmdTime = 0;
unsigned long lastWatchdogCheck = 0;

char     rxBuffer[80];
uint8_t  rxIndex = 0;

void brakeLeft() {
  digitalWrite(PIN_L_PWM, 0);
  digitalWrite(PIN_L_BRAKE, BRAKE_ACTIVE_LEVEL);
}
void brakeRight() {
  digitalWrite(PIN_R_PWM, 0);
  digitalWrite(PIN_R_BRAKE, BRAKE_ACTIVE_LEVEL);
}
void brakeAll() {
  brakeLeft();
  brakeRight();
  digitalWrite(PIN_STATUS_LED, HIGH);
}

void estopISR() {
  estop = true;                              // toujours sûr : un ISR ne peut que déclencher l'arrêt
}

bool parseIntField(char *&p, int &out) {
  char *end;
  long v = strtol(p, &end, 10);
  if (end == p) return false;                // pas un nombre
  out = (int)v;
  p = end;
  if (*p == ',') p++;                        // saute la virgule suivante
  return true;
}

void processLine(char *line) {
  if (line[0] == 'M' && line[1] == ',') {
    int lpwm, ldir, lbrk, rpwm, rdir, rbrk;
    char *p = line + 2;
    if (!(parseIntField(p, lpwm) && parseIntField(p, ldir) && parseIntField(p, lbrk) &&
          parseIntField(p, rpwm) && parseIntField(p, rdir) && parseIntField(p, rbrk))) {
      brakeAll();
      Serial.println("ERR:PARSE");
      return;
    }
    // validation stricte des plages
    if (lpwm < 0 || lpwm > 255 || rpwm < 0 || rpwm > 255 ||
        ldir < 0 || ldir > 1 || rdir < 0 || rdir > 1 ||
        lbrk < 0 || lbrk > 1 || rbrk < 0 || rbrk > 1) {
      brakeAll();
      Serial.println("ERR:RANGE");
      return;
    }
    lastCmdTime = millis();
    digitalWrite(PIN_STATUS_LED, LOW);

    if (lbrk == 1) brakeLeft();
    else {
      digitalWrite(PIN_L_BRAKE, !BRAKE_ACTIVE_LEVEL);
      digitalWrite(PIN_L_DIR, ldir == 1 ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
      analogWrite(PIN_L_PWM, lpwm);
    }
    if (rbrk == 1) brakeRight();
    else {
      digitalWrite(PIN_R_BRAKE, !BRAKE_ACTIVE_LEVEL);
      digitalWrite(PIN_R_DIR, rdir == 1 ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
      analogWrite(PIN_R_PWM, rpwm);
    }
    return;
  }
  if (line[0] == 'S' && line[1] == '\0') {
    serialEstop = true; estop = true; brakeAll();
    Serial.println("ESTOP:ON");
    return;
  }
  if (line[0] == 'C' && line[1] == '\0') {
    serialEstop = false; estop = digitalRead(PIN_ESTOP_BUTTON) == LOW;
    if (!estop) digitalWrite(PIN_STATUS_LED, LOW);
    Serial.println(estop ? "ESTOP:ON" : "ESTOP:OFF");
    return;
  }
  if (line[0] == 'P' && line[1] == '\0') {
    Serial.println("OK");
    return;
  }
  brakeAll();
  Serial.println("ERR:UNKNOWN");
}

void setup() {
  pinMode(PIN_R_BRAKE, OUTPUT);
  pinMode(PIN_R_DIR,   OUTPUT);
  pinMode(PIN_R_PWM,   OUTPUT);
  pinMode(PIN_L_BRAKE, OUTPUT);
  pinMode(PIN_L_DIR,   OUTPUT);
  pinMode(PIN_L_PWM,   OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);

  // BOUTON URGENCE (optionnel) : entre broche 2 et GND
  pinMode(PIN_ESTOP_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP_BUTTON), estopISR, FALLING);
  estop = (digitalRead(PIN_ESTOP_BUTTON) == LOW);

  // ÉTAT SÛR AU DÉMARRAGE : moteurs freinés, jamais de dém automatique
  brakeAll();
  Serial.begin(115200);
  while (!Serial) { ; }                      // attend l'USB (Leonardo) ; inoffensif sur UNO
  Serial.println("READY");
}

void loop() {
  // 1) lecture série
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      rxBuffer[rxIndex] = '\0';
      if (rxIndex > 0) processLine(rxBuffer);
      rxIndex = 0;
    } else if (c != '\r') {
      if (rxIndex < sizeof(rxBuffer) - 1) rxBuffer[rxIndex++] = c;
      else { rxIndex = 0; brakeAll(); Serial.println("ERR:OVERFLOW"); }
    }
  }

  // 2) watchdog + emergency stop (vérifiés à 100 Hz)
  unsigned long now = millis();
  if (now - lastWatchdogCheck >= WATCHDOG_CHECK_MS) {
    lastWatchdogCheck = now;
    if (estop || serialEstop) {
      brakeAll();
    } else if (now - lastCmdTime > CMD_TIMEOUT_MS) {
      // communication perdue (Pi planté, USB débranché, ROS arrêté) -> BRAKE
      brakeAll();
    }
  }
}
