#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

// I2C LCD (adres 0x27, 16 sütun x 2 satır)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID modül pinleri
#define SS_PIN 53
#define RST_PIN 49
MFRC522 rfid(SS_PIN, RST_PIN);

// Pin tanımlamaları
const int buttonPin = 10;
const int irAnalogPin = A0;
const int RPWM = 5;
const int LPWM = 6;
const int REN = 7;
const int LEN = 8;

// Stepper Motor pinleri (NEMA 17)
#define DIR_PIN 2
#define PUL_PIN 4
#define DIR_BUTTON_PIN 3
#define START_STOP_PIN 9

// Stepper motor değişkenleri - interrupt için volatile
volatile bool motorDirection = true;
volatile bool motorRunning = false;
volatile bool pulseState = false;

// Stepper button variables (Manual Control)
bool lastDirButtonState = HIGH;
unsigned long lastDirDebounceTime = 0;
bool lastStartStopState = HIGH;
unsigned long lastStartStopDebounceTime = 0;
unsigned long debounceDelay = 50; // ms

// Sistem değişkenleri
// IMPORTANT: Calibrate this threshold!
// If LOW IR reading = NO obstacle, and HIGH IR reading = OBSTACLE,
// set this threshold BETWEEN those typical readings.
// Example: If no obstacle reads ~50, and obstacle reads ~350, a threshold of 200 might work.
const int irThreshold = 150; // <<< !!! CALIBRATE THIS VALUE !!!
const unsigned long runDurationRamp = 2500;
const unsigned long runDurationStepper = 14000;
const int motorSpeedRamp = 200;
const unsigned long obstacleTimeout = 5000;
const unsigned long rfidReadCooldown = 2000;
const unsigned long cardProcessingDisplayTime = 1500;
const unsigned long invalidCardDisplayTime = 2000;

// --- RFID Yetkilendirme Listesi ---
String authorizedUids[] = {
  "D32822DA",
  "11223344"
};
const int numAuthorizedUids = sizeof(authorizedUids) / sizeof(authorizedUids[0]);
// ---------------------------------

enum SystemState {
  STOPPED,
  CARD_READ_PROCESSING,
  INVALID_CARD_DETECTED,
  STEPPER_MOVING_FORWARD,
  OPENING_RAMP,
  CLOSING_RAMP,
  STEPPER_MOVING_BACKWARD
};
SystemState currentState = STOPPED;

bool rampOpen = false;
bool rampOpenedByRFIDGlobal = false;
bool isForwardSequenceInitiatedByRFID = false;

unsigned long actionStartTimestamp = 0;
unsigned long lastObstacleTimestamp = 0;
bool obstacleDetectedCurrent = false;
unsigned long lastSuccessfulRFIDReadTime = 0;
String lastReadCardUID = "";

void stepperTimerISR() {
  if (motorRunning) {
    if (pulseState) {
      digitalWrite(PUL_PIN, HIGH);
      pulseState = false;
    } else {
      digitalWrite(PUL_PIN, LOW);
      pulseState = true;
    }
  } else {
    digitalWrite(PUL_PIN, LOW);
  }
}

void printUID(MFRC522::Uid uid) {
  Serial.print(F("Card UID: "));
  for (byte i = 0; i < uid.size; i++) {
    Serial.print(uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(uid.uidByte[i], HEX);
  }
  Serial.println();
}

String getUIDString(MFRC522::Uid uid) {
  String uidStr = "";
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) uidStr += "0";
    uidStr += String(uid.uidByte[i], HEX);
  }
  uidStr.toUpperCase();
  return uidStr;
}

bool isCardAuthorized(String uid) {
  for (int i = 0; i < numAuthorizedUids; i++) {
    if (uid == authorizedUids[i]) {
      return true;
    }
  }
  return false;
}

void updateLCD() {
  static String prevLine1 = "";
  static String prevLine2 = "";
  String currentLine1 = "", currentLine2 = "";
  unsigned long now = millis();

  bool isActiveOperationState = (currentState == OPENING_RAMP ||
                                 currentState == CLOSING_RAMP ||
                                 currentState == STEPPER_MOVING_FORWARD ||
                                 currentState == STEPPER_MOVING_BACKWARD ||
                                 currentState == CARD_READ_PROCESSING);

  if (obstacleDetectedCurrent && isActiveOperationState) {
    currentLine1 = "Engel Algilandi!";
    currentLine2 = "Yol Temizleyin";
  } else {
    switch (currentState) {
      case OPENING_RAMP:
        currentLine1 = "Rampa Aciliyor";
        currentLine2 = "Lutfen Bekleyin";
        break;
      case CLOSING_RAMP:
        currentLine1 = "Rampa Kapaniyor";
        currentLine2 = "Stepper Bekliyor";
        break;
      case CARD_READ_PROCESSING:
        currentLine1 = "Kart Okundu";
        currentLine2 = "Islem Yapiliyor..";
        break;
      case INVALID_CARD_DETECTED:
        if (obstacleDetectedCurrent) {
          currentLine1 = "Engel Algilandi!";
          currentLine2 = "Giris Engellendi";
        } else {
          currentLine1 = "Gecersiz Kart";
          currentLine2 = "Lutfen Cikarin";
        }
        break;
      case STEPPER_MOVING_FORWARD:
        currentLine1 = "Stepper Ileri";
        currentLine2 = motorRunning ? "Rampa Bekliyor" : "NEMA Durdu (Mnl)";
        break;
      case STEPPER_MOVING_BACKWARD:
        currentLine1 = "Stepper Geri";
        currentLine2 = motorRunning ? "Islem Suruyor" : "NEMA Durdu (Mnl)";
        break;
      case STOPPED:
        if (obstacleDetectedCurrent) {
          currentLine1 = "Engel Algilandi!";
          if (lastObstacleTimestamp > 0 && (now - lastObstacleTimestamp < obstacleTimeout)) {
            currentLine2 = "Bekleniyor...";
          } else {
            currentLine2 = "Yol Kapali";
          }
        } else if (rampOpen) {
          currentLine1 = "Rampa Acik";
          currentLine2 = rampOpenedByRFIDGlobal ? "RFID ile Acildi" : "Dugmeyle Acildi";
        } else {
          currentLine1 = "Rampa Kapali";
          currentLine2 = "Giris Bekliyor";
        }
        break;
      default:
        currentLine1 = "Sistem Hatasi";
        currentLine2 = "Yeniden Baslatin";
        break;
    }
  }

  if (currentLine1 != prevLine1 || currentLine2 != prevLine2) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(currentLine1);
    lcd.setCursor(0,1); lcd.print(currentLine2);
    prevLine1 = currentLine1;
    prevLine2 = currentLine2;
    Serial.print(F("LCD Update: L1='")); Serial.print(currentLine1);
    Serial.print(F("', L2='")); Serial.print(currentLine2); Serial.println(F("'"));
  }
}

void stopAllMotorsAndGoToStopped(const char* reason) {
  Serial.print(F("SYSTEM_STOP: Reason: ")); Serial.println(reason);
  motorRunning = false;
  digitalWrite(PUL_PIN, LOW);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  currentState = STOPPED;
  rampOpenedByRFIDGlobal = false;
  isForwardSequenceInitiatedByRFID = false;
  Serial.println(F("SYS_STATE: -> STOPPED (All motors off)"));
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println(F("========================================="));
  Serial.println(F("Enhanced Ramp + Stepper System V9 Starting..."));
  Serial.println(F("========================================="));
  Serial.print(F("IR Threshold set to: ")); Serial.println(irThreshold);
  Serial.println(F("Ensure IR sensor is calibrated: Low reading = NO obstacle, High reading = OBSTACLE"));


  SPI.begin();
  Serial.println(F("- SPI Initialized."));

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(irAnalogPin, INPUT);
  pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
  pinMode(REN,  OUTPUT); pinMode(LEN,  OUTPUT);
  digitalWrite(REN, HIGH); digitalWrite(LEN, HIGH);
  Serial.println(F("- DC Motor (Ramp) & IR Sensor Pins Initialized."));

  pinMode(DIR_PIN, OUTPUT); pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_STOP_PIN, INPUT_PULLUP);
  motorDirection = true;
  digitalWrite(DIR_PIN, motorDirection ? HIGH : LOW);
  digitalWrite(PUL_PIN, LOW);
  Serial.println(F("- Stepper Motor Pins Initialized (Default Forward)."));

  Timer1.initialize(200);
  Timer1.attachInterrupt(stepperTimerISR);
  Serial.println(F("- Timer1 (Stepper Pulse) Initialized: 200us interval."));

  Wire.begin(); lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("Sistem Basliyor"));
  Serial.println(F("- LCD Initialized."));
  delay(1000);

  rfid.PCD_Init(); delay(50);
  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print(F("- RFID MFRC522 Version: 0x")); Serial.println(version, HEX);
  if (version != 0x00 && version != 0xFF) {
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
    Serial.println(F("- RFID Antenna Gain set to Max."));
  } else {
    Serial.println(F("WARNING: MFRC522 not found!"));
    lcd.clear(); lcd.print(F("RFID HATA!")); lcd.setCursor(0,1); lcd.print(F("Kontrol Edin"));
  }

  updateLCD();
  Serial.println(F("========================================="));
  Serial.println(F("System Ready. Waiting for input..."));
  Serial.println(F("========================================="));
}

void loop() {
  unsigned long now = millis();

  handleStepperButtonsManual(now);

  static unsigned long lastIRCheckTime = 0;
  if (now - lastIRCheckTime > 100) {
    int irReading = analogRead(irAnalogPin);
    bool prevObstacleState = obstacleDetectedCurrent;
    
    // CORRECTED LOGIC: Obstacle if reading is GREATER than threshold
    obstacleDetectedCurrent = (irReading > irThreshold); 

    if (obstacleDetectedCurrent && !prevObstacleState) {
      lastObstacleTimestamp = now;
      Serial.print(F("IR LOGIC: Obstacle DETECTED. Analog: ")); Serial.print(irReading);
      Serial.print(F(" (Threshold: ")); Serial.print(irThreshold); Serial.println(F(")"));
      bool isActiveMotorOpState = (currentState == OPENING_RAMP ||
                                   currentState == CLOSING_RAMP ||
                                   currentState == STEPPER_MOVING_FORWARD ||
                                   currentState == STEPPER_MOVING_BACKWARD);
      if (isActiveMotorOpState) {
        stopAllMotorsAndGoToStopped("Obstacle Detected During Motor Operation");
      }
    } else if (!obstacleDetectedCurrent && prevObstacleState) {
      Serial.print(F("IR LOGIC: Obstacle CLEARED. Analog: ")); Serial.print(irReading);
      Serial.print(F(" (Threshold: ")); Serial.print(irThreshold); Serial.println(F(")"));
    }
    lastIRCheckTime = now;
  }

  static unsigned long lastIRPeriodicLogTime = 0;
  if (now - lastIRPeriodicLogTime >= 1000) {
    int currentIRReadingForLog = analogRead(irAnalogPin);
    // CORRECTED LOGIC for periodic log too
    bool isObstacleForLog = (currentIRReadingForLog > irThreshold); 
    Serial.print(F("IR STATUS (1s): Value = "));
    Serial.print(currentIRReadingForLog);
    if (isObstacleForLog) {
      Serial.println(F(" - Obstacle DETECTED"));
    } else {
      Serial.println(F(" - Path clear"));
    }
    lastIRPeriodicLogTime = now;
  }

  bool canSystemAutomatedSequencesOperate = !obstacleDetectedCurrent;

  static unsigned long lastRFIDResetTime = 0;
  if (now - lastRFIDResetTime > 60000) {
    Serial.println(F("RFID: Performing periodic PCD_Reset and PCD_Init."));
    rfid.PCD_Reset(); delayMicroseconds(100); rfid.PCD_Init(); rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
    lastRFIDResetTime = now; Serial.println(F("RFID: Periodic reset complete."));
  }

  static unsigned long lastRFIDCheckTime = 0;
  if ((currentState == STOPPED || currentState == INVALID_CARD_DETECTED) && !rampOpen && (now - lastRFIDCheckTime > 200)) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      String currentCardUID = getUIDString(rfid.uid);
      Serial.print(F("RFID: New card detected. ")); printUID(rfid.uid);

      if (now - lastSuccessfulRFIDReadTime < rfidReadCooldown && currentCardUID == lastReadCardUID) {
        Serial.println(F("RFID: Same card read within cooldown. Ignoring."));
      } else {
        lastSuccessfulRFIDReadTime = now;
        lastReadCardUID = currentCardUID;

        if (isCardAuthorized(currentCardUID)) {
          if (canSystemAutomatedSequencesOperate) {
            Serial.println(F("RFID: Authorized card. Initiating sequence."));
            isForwardSequenceInitiatedByRFID = true;
            currentState = CARD_READ_PROCESSING;
            actionStartTimestamp = now;
            Serial.println(F("SYS_STATE: -> CARD_READ_PROCESSING (By RFID)"));
          } else {
            Serial.println(F("RFID: Authorized card, but OBSTACLE. Operation blocked."));
            currentState = INVALID_CARD_DETECTED;
            actionStartTimestamp = now;
          }
        } else {
          Serial.println(F("RFID: Unauthorized card."));
          currentState = INVALID_CARD_DETECTED;
          actionStartTimestamp = now;
        }
      }
      rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
    }
    lastRFIDCheckTime = now;
  }

  static unsigned long lastMainButtonCheckTime = 0;
  static bool prevMainButtonState = HIGH;
  if (now - lastMainButtonCheckTime > 50) {
    bool currentMainButtonState = digitalRead(buttonPin);
    if (currentMainButtonState == LOW && prevMainButtonState == HIGH) {
      Serial.println(F("MAIN_BTN(10): Pressed."));
      if (currentState == STOPPED) {
        if (canSystemAutomatedSequencesOperate) {
          if (!rampOpen) {
            Serial.println(F("MAIN_BTN(10): Initiating FORWARD sequence."));
            isForwardSequenceInitiatedByRFID = false;
            // Go to CARD_READ_PROCESSING to unify start, it will then check obstacle again before motor
            currentState = CARD_READ_PROCESSING; 
            actionStartTimestamp = now;
          } else {
            Serial.println(F("MAIN_BTN(10): Initiating REVERSE sequence."));
            currentState = CLOSING_RAMP;
            actionStartTimestamp = now;
          }
        } else {
          Serial.println(F("MAIN_BTN(10): Pressed, but OBSTACLE. Operation blocked."));
          currentState = INVALID_CARD_DETECTED;
          actionStartTimestamp = now;
        }
      } else {
         stopAllMotorsAndGoToStopped("Main Button (10) pressed during active/error state");
      }
    }
    prevMainButtonState = currentMainButtonState;
    lastMainButtonCheckTime = now;
  }

  switch (currentState) {
    case CARD_READ_PROCESSING:
      if (now - actionStartTimestamp > cardProcessingDisplayTime) {
        if (!obstacleDetectedCurrent) {
            currentState = STEPPER_MOVING_FORWARD;
            actionStartTimestamp = now;
            motorDirection = true;
            digitalWrite(DIR_PIN, HIGH);
            motorRunning = true;
            Serial.println(F("SYS_STATE: CARD_READ_PROCESSING -> STEPPER_MOVING_FORWARD"));
            Serial.println(F("STEPPER_CTRL: NEMA motor FORWARD sequence initiated."));
        } else {
            Serial.println(F("SYS_STATE: CARD_READ_PROCESSING -> OBSTACLE DETECTED. Blocking operation."));
            currentState = INVALID_CARD_DETECTED;
            actionStartTimestamp = now;
        }
      }
      analogWrite(RPWM, 0); analogWrite(LPWM, 0);
      break;

    case INVALID_CARD_DETECTED:
      if (now - actionStartTimestamp > invalidCardDisplayTime) {
        currentState = STOPPED;
        Serial.println(F("SYS_STATE: INVALID_CARD_DETECTED -> STOPPED (Display time elapsed)"));
      }
      analogWrite(RPWM, 0); analogWrite(LPWM, 0);
      break;

    case STEPPER_MOVING_FORWARD:
      if ((now - actionStartTimestamp) >= runDurationStepper) {
        Serial.println(F("STEPPER_CTRL: NEMA FORWARD time elapsed."));
        motorRunning = false;
        digitalWrite(PUL_PIN, LOW);
        if (!obstacleDetectedCurrent) {
            currentState = OPENING_RAMP;
            actionStartTimestamp = now;
            Serial.print(F("SYS_STATE: STEPPER_MOVING_FORWARD -> OPENING_RAMP (Initiated by "));
            Serial.println(isForwardSequenceInitiatedByRFID ? F("RFID)") : F("Button)"));
        } else {
            Serial.println(F("SYS_STATE: STEPPER_MOVING_FORWARD -> OBSTACLE. Halting."));
            stopAllMotorsAndGoToStopped("Obstacle after NEMA fwd, before ramp open");
        }
      }
      analogWrite(RPWM, 0); analogWrite(LPWM, 0);
      break;

    case OPENING_RAMP:
      if ((now - actionStartTimestamp) < runDurationRamp) {
        analogWrite(RPWM, motorSpeedRamp); analogWrite(LPWM, 0);
      } else {
        Serial.println(F("RAMP_CTRL: Ramp OPENED."));
        analogWrite(RPWM, 0); analogWrite(LPWM, 0);
        rampOpen = true;
        rampOpenedByRFIDGlobal = isForwardSequenceInitiatedByRFID;
        currentState = STOPPED;
        Serial.print(F("SYS_STATE: OPENING_RAMP -> STOPPED. Ramp opened by RFID: "));
        Serial.println(rampOpenedByRFIDGlobal ? F("true") : F("false"));
      }
      break;

    case CLOSING_RAMP:
      if ((now - actionStartTimestamp) < runDurationRamp) {
        analogWrite(RPWM, 0); analogWrite(LPWM, motorSpeedRamp);
      } else {
        Serial.println(F("RAMP_CTRL: Ramp CLOSED."));
        analogWrite(RPWM, 0); analogWrite(LPWM, 0);
        rampOpen = false;
        if (!obstacleDetectedCurrent) {
            currentState = STEPPER_MOVING_BACKWARD;
            actionStartTimestamp = now;
            motorDirection = false;
            digitalWrite(DIR_PIN, LOW);
            motorRunning = true;
            Serial.println(F("SYS_STATE: CLOSING_RAMP -> STEPPER_MOVING_BACKWARD"));
            Serial.println(F("STEPPER_CTRL: NEMA motor REVERSE sequence initiated."));
        } else {
            Serial.println(F("SYS_STATE: CLOSING_RAMP -> OBSTACLE. Halting."));
            stopAllMotorsAndGoToStopped("Obstacle after ramp close, before NEMA bwd");
        }
      }
      break;

    case STEPPER_MOVING_BACKWARD:
      if ((now - actionStartTimestamp) >= runDurationStepper) {
        Serial.println(F("STEPPER_CTRL: NEMA REVERSE time elapsed."));
        motorRunning = false;
        digitalWrite(PUL_PIN, LOW);
        currentState = STOPPED;
        rampOpenedByRFIDGlobal = false;
        Serial.println(F("SYS_STATE: STEPPER_MOVING_BACKWARD -> STOPPED (Reverse Sequence Complete)"));
      }
      analogWrite(RPWM, 0); analogWrite(LPWM, 0);
      break;

    case STOPPED:
    default:
      if (digitalRead(RPWM) != 0 || digitalRead(LPWM) != 0) {
        analogWrite(RPWM, 0); analogWrite(LPWM, 0);
      }
      break;
  }

  static unsigned long lastLCDUpdateTime = 0;
  if (now - lastLCDUpdateTime > 250) {
    updateLCD();
    lastLCDUpdateTime = now;
  }

  static unsigned long lastStatusReportTime = 0;
  if (now - lastStatusReportTime > 20000) {
    Serial.println(F("--- Status Report (20s) ---"));
    Serial.print(F("  Timestamp: ")); Serial.println(now);
    Serial.print(F("  System State: "));
    switch(currentState) {
      case STOPPED: Serial.println(F("STOPPED")); break;
      case CARD_READ_PROCESSING: Serial.println(F("CARD_READ_PROCESSING")); break;
      case INVALID_CARD_DETECTED: Serial.println(F("INVALID_CARD_DETECTED")); break;
      case STEPPER_MOVING_FORWARD: Serial.println(F("STEPPER_MOVING_FORWARD")); break;
      case OPENING_RAMP: Serial.println(F("OPENING_RAMP")); break;
      case CLOSING_RAMP: Serial.println(F("CLOSING_RAMP")); break;
      case STEPPER_MOVING_BACKWARD: Serial.println(F("STEPPER_MOVING_BACKWARD")); break;
      default: Serial.println(F("UNKNOWN")); break;
    }
    Serial.print(F("  Ramp Physically Open: ")); Serial.println(rampOpen ? "Yes" : "No");
    Serial.print(F("  Obstacle Detected: ")); Serial.println(obstacleDetectedCurrent ? "Yes" : "No");
    if (obstacleDetectedCurrent && lastObstacleTimestamp > 0) {
      Serial.print(F("  Time Since Obstacle First Noted: ")); Serial.print(now - lastObstacleTimestamp); Serial.println(F(" ms"));
    }
    Serial.print(F("  Can Auto Sequences Operate: ")); Serial.println(canSystemAutomatedSequencesOperate ? "Yes" : "No");
    Serial.print(F("  NEMA Motor Running: ")); Serial.println(motorRunning ? "Yes" : "No");
    Serial.print(F("  NEMA Motor Direction (true=fwd): ")); Serial.println(motorDirection);
    Serial.println(F("---------------------------"));
    lastStatusReportTime = now;
  }
}

void handleStepperButtonsManual(unsigned long currentTime) {
  static unsigned long lastStepperBtnCheckTime = 0;
  static bool currentDirButtonStateLocal = HIGH;
  static bool currentStartStopStateLocal = HIGH;

  if (currentTime - lastStepperBtnCheckTime < 20) return;
  lastStepperBtnCheckTime = currentTime;

  int startStopReading = digitalRead(START_STOP_PIN);
  if (startStopReading != lastStartStopState) {
    lastStartStopDebounceTime = currentTime;
  }
  if ((currentTime - lastStartStopDebounceTime) > debounceDelay) {
    if (startStopReading != currentStartStopStateLocal) {
      currentStartStopStateLocal = startStopReading;
      if (currentStartStopStateLocal == LOW) {
        motorRunning = !motorRunning;
        Serial.print(F("MANUAL_STEPPER_BTN(9): NEMA Start/Stop. NEMA Motor Running is now: ")); Serial.println(motorRunning ? "ON" : "OFF");
        if (!motorRunning) {
          digitalWrite(PUL_PIN, LOW);
        }
      }
    }
  }
  lastStartStopState = startStopReading;

  int dirReading = digitalRead(DIR_BUTTON_PIN);
  if (dirReading != lastDirButtonState) {
    lastDirDebounceTime = currentTime;
  }
  if ((currentTime - lastDirDebounceTime) > debounceDelay) {
    if (dirReading != currentDirButtonStateLocal) {
      currentDirButtonStateLocal = dirReading;
      if (currentDirButtonStateLocal == LOW) {
        motorDirection = !motorDirection;
        digitalWrite(DIR_PIN, motorDirection ? HIGH : LOW);
        Serial.print(F("MANUAL_STEPPER_BTN(3): NEMA Direction Changed. NEMA Dir is now: ")); Serial.println(motorDirection ? "FWD (DIR_PIN=HIGH)" : "BWD (DIR_PIN=LOW)");
      }
    }
  }
  lastDirButtonState = dirReading;
}