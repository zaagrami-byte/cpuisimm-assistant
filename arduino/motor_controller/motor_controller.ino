/**
 * ISIMM ROBOT — Arduino UNO : contrôle bas niveau des 2 moteurs hoverboard.
 *
 * AUCUN encodeur.
 *
 * Cette carte fait uniquement :
 *   - recevoir les commandes série de la Raspberry Pi ;
 *   - générer PWM / DIR / BRAKE ;
 *   - surveiller la communication (watchdog) ;
 *   - gérer l'arrêt d'urgence.
 *
 * IMPORTANT :
 * Toutes les sorties sont commandées avec analogWrite().
 *
 *   analogWrite(pin, 0)   = LOW
 *   analogWrite(pin, 255) = HIGH
 *
 *
 * PROTOCOLE (115200 baud, '\n' en fin de ligne) :
 *
 *   M,<lpwm>,<ldir>,<lbrk>,<rpwm>,<rdir>,<rbrk>
 *
 *   pwm : 0..255
 *   dir : 0/1
 *   brk : 0/1
 *
 *   S -> emergency stop (verrouillé)
 *   C -> clear emergency stop
 *   P -> ping (répond "OK")
 *
 * Boot :
 *   READY
 *
 * Toute commande invalide :
 *   moteurs STOP + réponse ERR
 *
 * Watchdog :
 *   si aucune commande M valide pendant CMD_TIMEOUT_MS
 *   -> BRAKE
 */

// ============================================================================
// RÉGLAGES MATÉRIELS
// ============================================================================

// ---------- MOTEUR DROIT ----------
const uint8_t PIN_R_BRAKE = 8;
const uint8_t PIN_R_DIR   = 9;
const uint8_t PIN_R_PWM   = 10;

// ---------- MOTEUR GAUCHE ----------
const uint8_t PIN_L_BRAKE = 7;
const uint8_t PIN_L_DIR   = 6;
const uint8_t PIN_L_PWM   = 5;


// ============================================================================
// POLARITÉ DES SIGNAUX
// ============================================================================

// Niveau analogique qui active le frein
//
// 255 = HIGH
//   0 = LOW
//
const uint8_t BRAKE_ACTIVE_LEVEL = 255;

// Niveau analogique DIR correspondant au sens "avant"
//
// 255 = HIGH
//   0 = LOW
//
const uint8_t DIR_FORWARD_LEVEL = 255;


// ============================================================================
// INVERSION DE SENS DES MOTEURS
// ============================================================================
//
// IMPORTANT :
// On ne change PAS les fils du moteur brushless.
//
// Si un moteur tourne dans le mauvais sens par rapport à l'autre,
// on inverse simplement sa commande DIR ici.
//
// Actuellement :
//   Gauche = normal
//   Droite = inversé
//
// Si le mauvais moteur est finalement le gauche :
//
//   INVERT_LEFT_DIR  = true;
//   INVERT_RIGHT_DIR = false;
//

const bool INVERT_LEFT_DIR  = false;
const bool INVERT_RIGHT_DIR = true;


// ============================================================================
// ARRÊT D'URGENCE
// ============================================================================

// Bouton physique optionnel : bouton entre PIN 2 et GND
const uint8_t PIN_ESTOP_BUTTON = 2;

// LED de statut
//
// 255 = robot en sécurité / arrêté
//   0 = commande moteur active
//
const uint8_t PIN_STATUS_LED = 4;


// ============================================================================
// WATCHDOG
// ============================================================================

// Temps maximum sans recevoir une commande M valide
const unsigned long CMD_TIMEOUT_MS = 500;

// Fréquence de vérification du watchdog
const unsigned long WATCHDOG_CHECK_MS = 10;


// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

// E-stop déclenché
volatile bool estop = false;

// E-stop déclenché par la commande série S
bool serialEstop = false;

// Temps de dernière commande moteur valide
unsigned long lastCmdTime = 0;

// Dernière vérification watchdog
unsigned long lastWatchdogCheck = 0;

// Buffer réception série
char rxBuffer[80];
uint8_t rxIndex = 0;


// ============================================================================
// FREIN MOTEUR GAUCHE
// ============================================================================

void brakeLeft() {

  // PWM à zéro
  analogWrite(PIN_L_PWM, 0);

  // Activation du frein
  analogWrite(
    PIN_L_BRAKE,
    BRAKE_ACTIVE_LEVEL
  );
}


// ============================================================================
// FREIN MOTEUR DROIT
// ============================================================================

void brakeRight() {

  // PWM à zéro
  analogWrite(PIN_R_PWM, 0);

  // Activation du frein
  analogWrite(
    PIN_R_BRAKE,
    BRAKE_ACTIVE_LEVEL
  );
}


// ============================================================================
// FREIN DES DEUX MOTEURS
// ============================================================================

void brakeAll() {

  brakeLeft();
  brakeRight();

  // LED = sécurité / arrêt
  analogWrite(
    PIN_STATUS_LED,
    255
  );
}


// ============================================================================
// INTERRUPTION E-STOP
// ============================================================================

void estopISR() {

  // L'ISR ne fait qu'activer l'arrêt d'urgence.
  // Le freinage réel est effectué dans loop().
  estop = true;
}


// ============================================================================
// LECTURE D'UN CHAMP NUMÉRIQUE
// ============================================================================

bool parseIntField(char *&p, int &out) {

  char *end;

  long v = strtol(p, &end, 10);

  // Aucun nombre trouvé
  if (end == p) {
    return false;
  }

  out = (int)v;

  p = end;

  // Passer la virgule suivante
  if (*p == ',') {
    p++;
  }

  return true;
}


// ============================================================================
// TRAITEMENT D'UNE LIGNE REÇUE
// ============================================================================

void processLine(char *line) {

  // ========================================================================
  // COMMANDE MOTEURS
  // ========================================================================

  if (line[0] == 'M' && line[1] == ',') {

    int lpwm;
    int ldir;
    int lbrk;

    int rpwm;
    int rdir;
    int rbrk;

    char *p = line + 2;


    // ----------------------------------------------------------------------
    // PARSING
    // ----------------------------------------------------------------------

    if (!(
      parseIntField(p, lpwm) &&
      parseIntField(p, ldir) &&
      parseIntField(p, lbrk) &&
      parseIntField(p, rpwm) &&
      parseIntField(p, rdir) &&
      parseIntField(p, rbrk)
    )) {

      brakeAll();

      Serial.println("ERR:PARSE");

      return;
    }


    // ----------------------------------------------------------------------
    // VALIDATION DES VALEURS
    // ----------------------------------------------------------------------

    if (
      lpwm < 0 || lpwm > 255 ||
      rpwm < 0 || rpwm > 255 ||

      ldir < 0 || ldir > 1 ||
      rdir < 0 || rdir > 1 ||

      lbrk < 0 || lbrk > 1 ||
      rbrk < 0 || rbrk > 1
    ) {

      brakeAll();

      Serial.println("ERR:RANGE");

      return;
    }


    // ----------------------------------------------------------------------
    // SI E-STOP ACTIF
    // ----------------------------------------------------------------------

    if (estop || serialEstop) {

      brakeAll();

      Serial.println("ERR:ESTOP");

      return;
    }


    // ----------------------------------------------------------------------
    // COMMANDE VALIDE
    // ----------------------------------------------------------------------

    lastCmdTime = millis();

    // LED = robot commandé
    analogWrite(
      PIN_STATUS_LED,
      0
    );


    // ======================================================================
    // MOTEUR GAUCHE
    // ======================================================================

    if (lbrk == 1) {

      // --------------------------------------------------------------
      // FREIN DEMANDÉ
      // --------------------------------------------------------------

      brakeLeft();

    } else {

      // --------------------------------------------------------------
      // DÉSACTIVATION DU FREIN
      // --------------------------------------------------------------

      analogWrite(
        PIN_L_BRAKE,
        0
      );


      // --------------------------------------------------------------
      // CALCUL DU SENS GAUCHE
      // --------------------------------------------------------------

      bool leftForward = (ldir == 1);


      // --------------------------------------------------------------
      // INVERSION LOGICIELLE
      // --------------------------------------------------------------

      if (INVERT_LEFT_DIR) {
        leftForward = !leftForward;
      }


      // --------------------------------------------------------------
      // APPLICATION DIR
      // --------------------------------------------------------------

      analogWrite(
        PIN_L_DIR,
        leftForward
          ? DIR_FORWARD_LEVEL
          : 0
      );


      // --------------------------------------------------------------
      // APPLICATION PWM
      // --------------------------------------------------------------

      analogWrite(
        PIN_L_PWM,
        lpwm
      );
    }


    // ======================================================================
    // MOTEUR DROIT
    // ======================================================================

    if (rbrk == 1) {

      // --------------------------------------------------------------
      // FREIN DEMANDÉ
      // --------------------------------------------------------------

      brakeRight();

    } else {

      // --------------------------------------------------------------
      // DÉSACTIVATION DU FREIN
      // --------------------------------------------------------------

      analogWrite(
        PIN_R_BRAKE,
        0
      );


      // --------------------------------------------------------------
      // CALCUL DU SENS DROIT
      // --------------------------------------------------------------

      bool rightForward = (rdir == 1);


      // --------------------------------------------------------------
      // INVERSION LOGICIELLE
      // --------------------------------------------------------------

      if (INVERT_RIGHT_DIR) {
        rightForward = !rightForward;
      }


      // --------------------------------------------------------------
      // APPLICATION DIR
      // --------------------------------------------------------------

      analogWrite(
        PIN_R_DIR,
        rightForward
          ? DIR_FORWARD_LEVEL
          : 0
      );


      // --------------------------------------------------------------
      // APPLICATION PWM
      // --------------------------------------------------------------

      analogWrite(
        PIN_R_PWM,
        rpwm
      );
    }

    return;
  }


  // ========================================================================
  // EMERGENCY STOP
  // ========================================================================

  if (line[0] == 'S' && line[1] == '\0') {

    serialEstop = true;
    estop = true;

    brakeAll();

    Serial.println("ESTOP:ON");

    return;
  }


  // ========================================================================
  // CLEAR EMERGENCY STOP
  // ========================================================================

  if (line[0] == 'C' && line[1] == '\0') {

    serialEstop = false;

    // Vérifier également le bouton physique
    estop = (
      digitalRead(PIN_ESTOP_BUTTON) == LOW
    );


    if (!estop) {

      analogWrite(
        PIN_STATUS_LED,
        0
      );

      Serial.println("ESTOP:OFF");

    } else {

      brakeAll();

      Serial.println("ESTOP:ON");
    }

    return;
  }


  // ========================================================================
  // PING
  // ========================================================================

  if (line[0] == 'P' && line[1] == '\0') {

    Serial.println("OK");

    return;
  }


  // ========================================================================
  // COMMANDE INCONNUE
  // ========================================================================

  brakeAll();

  Serial.println("ERR:UNKNOWN");
}


// ============================================================================
// SETUP
// ============================================================================

void setup() {

  // ========================================================================
  // CONFIGURATION DES BROCHES MOTEUR DROIT
  // ========================================================================

  pinMode(PIN_R_BRAKE, OUTPUT);
  pinMode(PIN_R_DIR,   OUTPUT);
  pinMode(PIN_R_PWM,   OUTPUT);


  // ========================================================================
  // CONFIGURATION DES BROCHES MOTEUR GAUCHE
  // ========================================================================

  pinMode(PIN_L_BRAKE, OUTPUT);
  pinMode(PIN_L_DIR,   OUTPUT);
  pinMode(PIN_L_PWM,   OUTPUT);


  // ========================================================================
  // LED
  // ========================================================================

  pinMode(PIN_STATUS_LED, OUTPUT);


  // ========================================================================
  // BOUTON E-STOP
  // ========================================================================

  // Bouton entre PIN 2 et GND
  pinMode(
    PIN_ESTOP_BUTTON,
    INPUT_PULLUP
  );


  attachInterrupt(
    digitalPinToInterrupt(PIN_ESTOP_BUTTON),
    estopISR,
    FALLING
  );


  // Lire l'état initial du bouton
  estop = (
    digitalRead(PIN_ESTOP_BUTTON) == LOW
  );


  // ========================================================================
  // ÉTAT SÛR AU DÉMARRAGE
  // ========================================================================

  brakeAll();


  // ========================================================================
  // COMMUNICATION SÉRIE
  // ========================================================================

  Serial.begin(115200);


  // Sur UNO classique, while(!Serial) ne bloque normalement pas.
  // Il est conservé pour compatibilité avec les cartes USB natives.

  while (!Serial) {
    ;
  }


  // ========================================================================
  // MESSAGE DE DÉMARRAGE
  // ========================================================================

  Serial.println("READY");
}


// ============================================================================
// LOOP
// ============================================================================

void loop() {

  // ========================================================================
  // 1. LECTURE SÉRIE
  // ========================================================================

  while (Serial.available() > 0) {

    char c = (char)Serial.read();


    // ----------------------------------------------------------------------
    // FIN DE LIGNE
    // ----------------------------------------------------------------------

    if (c == '\n') {

      rxBuffer[rxIndex] = '\0';


      if (rxIndex > 0) {

        processLine(rxBuffer);
      }


      rxIndex = 0;
    }


    // ----------------------------------------------------------------------
    // IGNORER CR
    // ----------------------------------------------------------------------

    else if (c != '\r') {

      // --------------------------------------------------------------------
      // AJOUT AU BUFFER
      // --------------------------------------------------------------------

      if (rxIndex < sizeof(rxBuffer) - 1) {

        rxBuffer[rxIndex++] = c;

      } else {

        // Buffer plein = sécurité
        rxIndex = 0;

        brakeAll();

        Serial.println("ERR:OVERFLOW");
      }
    }
  }


  // ========================================================================
  // 2. WATCHDOG + E-STOP
  // ========================================================================

  unsigned long now = millis();


  if (
    now - lastWatchdogCheck >= WATCHDOG_CHECK_MS
  ) {

    lastWatchdogCheck = now;


    // ----------------------------------------------------------------------
    // E-STOP
    // ----------------------------------------------------------------------

    if (estop || serialEstop) {

      brakeAll();
    }


    // ----------------------------------------------------------------------
    // PERTE DE COMMUNICATION
    // ----------------------------------------------------------------------

    else if (
      now - lastCmdTime > CMD_TIMEOUT_MS
    ) {

      // Raspberry Pi arrêtée,
      // ROS arrêté,
      // câble USB débranché,
      // node moteur arrêté, etc.
      //
      // => freinage immédiat.

      brakeAll();
    }
  }
}
