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

// YENİ EKLENEN: Lineer Aktüatör (Rampa) Manuel Kontrol Pinleri
const int rampDirButtonPin = 11;         // Lineer Aktüatör (Rampa) Yön Butonu
const int rampStartStopButtonPin = 12;   // Lineer Aktüatör (Rampa) Başlat/Durdur Butonu

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

// YENİ EKLENEN: Ramp manual control button state tracking (for debouncing) - Rampa manuel kontrol buton durum takibi (parazit önleme için)
bool lastRampDirButtonState_manual = HIGH;
unsigned long lastRampDirDebounceTime_manual = 0;
bool lastRampStartStopButtonState_manual = HIGH;
unsigned long lastRampStartStopDebounceTime_manual = 0;

// YENİ EKLENEN: Ramp manual control motor state variables - Rampa manuel kontrol motor durum değişkenleri
bool ramp_manualMotorRunning = false;             // Rampa motoru manuel olarak çalışıyor mu?
bool ramp_manualDirectionForward = true;          // Rampa manuel yönü: true = İLERİ (Açma), false = GERİ (Kapama)


// Sistem değişkenleri
const int irThreshold = 150; // <<< !!! CALIBRATE THIS VALUE !!!
const unsigned long runDurationRamp = 2500;
const unsigned long runDurationStepper = 14000;
const int motorSpeedRamp = 200;
const unsigned long obstacleTimeout = 5000;
const unsigned long rfidReadCooldown = 2000;
const unsigned long cardProcessingDisplayTime = 1500; // Used for "Kart Okundu" before "Yetkili Kart" takes over
const unsigned long invalidCardDisplayTime = 2000;

// --- RFID Yetkilendirme Listesi ---
String authorizedUids[] = {
  "D32822DA",
  "11223344"
  // Add your other UIDs here
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

// --- Variables for timer and resume functionality ---
unsigned long remainingOperationTime = 0;
SystemState interruptedState = STOPPED;
volatile bool wasStoppedByObstacle = false;
// ----------------------------------------------------

// --- Constants and Flags for User-Friendly LCD Messages ---
const unsigned long YETKILI_KART_DISPLAY_TIME = 2000; // 2 seconds for "Yetkili Kart"
const unsigned long BRIEF_MESSAGE_DURATION = 2000;    // 2 seconds for "Rampa Acildi/Kapandi"

bool showBriefMessageRampaAcildi = false;
bool showBriefMessageRampaKapandi = false;
unsigned long briefMessageDisplayUntil = 0;
// --------------------------------------------------------


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

  // --- Brief Timed Messages (Priority 1) ---
  if (showBriefMessageRampaAcildi && now < briefMessageDisplayUntil && !obstacleDetectedCurrent) {
    currentLine1 = "Rampa Acildi";
    currentLine2 = "Kullanilabilir";
  } else if (showBriefMessageRampaKapandi && now < briefMessageDisplayUntil && !obstacleDetectedCurrent) {
    currentLine1 = "Rampa Kapandi";
    currentLine2 = "Giris Bekliyor";
  } else {
    // Reset brief message flags if their time is up or an obstacle appears
    if (now >= briefMessageDisplayUntil || obstacleDetectedCurrent) {
        showBriefMessageRampaAcildi = false;
        showBriefMessageRampaKapandi = false;
    }

    // --- Obstacle Messages (Priority 2) ---
    if (obstacleDetectedCurrent) {
      currentLine1 = "Engel Algilandi!";
      if (currentState == STOPPED && wasStoppedByObstacle && !obstacleDetectedCurrent) {
          currentLine2 = "Devam icin Buton"; // Obstacle cleared, waiting for button
      } else if (currentState == STOPPED && wasStoppedByObstacle && obstacleDetectedCurrent) {
          currentLine2 = "Yolu Temizleyin";   // Obstacle still there
      } else if (currentState == INVALID_CARD_DETECTED && obstacleDetectedCurrent) {
          currentLine2 = "Giris Engellendi";
      } else {
          currentLine2 = "Yolu Temizleyin";   // General obstacle message
      }
    }
    // --- Resume Indication (Priority 3 - if no obstacle and no brief message) ---
    else if (currentState == STOPPED && wasStoppedByObstacle && !obstacleDetectedCurrent) {
      currentLine1 = "Engel Temizlendi";
      currentLine2 = "Devam icin Buton";
    }
    // --- Normal Operation Messages (Priority 4) ---
    else {
      switch (currentState) {
        case CARD_READ_PROCESSING:
          if (isForwardSequenceInitiatedByRFID && (now - actionStartTimestamp < YETKILI_KART_DISPLAY_TIME)) {
            currentLine1 = "Yetkili Kart";      // "Authorized Card"
            currentLine2 = "Islem Basliyor..."; // "Processing Starts..."
          } else {
            currentLine1 = "Rampa Aciliyor";    // "Ramp Opening"
            currentLine2 = "Lutfen Bekleyin";  // "Please Wait"
          }
          break;

        case STEPPER_MOVING_FORWARD: // Fall through
        case OPENING_RAMP:
          currentLine1 = "Rampa Aciliyor";      // "Ramp Opening"
          currentLine2 = wasStoppedByObstacle ? "Devam Ediyor..." : "Lutfen Bekleyin."; // "Resuming..." or "Please Wait."
          break;

        case CLOSING_RAMP: // Fall through
        case STEPPER_MOVING_BACKWARD:
          currentLine1 = "Rampa Kapaniyor";     // "Ramp Closing"
          currentLine2 = wasStoppedByObstacle ? "Devam Ediyor..." : "Lutfen Bekleyin."; // "Resuming..." or "Please Wait."
          break;

        case INVALID_CARD_DETECTED:
          // This is reached if no obstacle caused this state (handled by obstacle check above)
          currentLine1 = "Gecersiz Kart";       // "Invalid Card"
          currentLine2 = "Rampa Baslatilamiyor";     // "Please Remove" / "Cannot Start Ramp"
          break;

        case STOPPED:
          // Default STOPPED messages if no other higher priority message is active
          // Also, do not show default STOPPED if manual ramp control is active
          if (ramp_manualMotorRunning) {
             currentLine1 = "Manuel Rampa";
             currentLine2 = ramp_manualDirectionForward ? "Aciliyor..." : "Kapaniyor...";
          } else if (rampOpen) {
            currentLine1 = "Rampa Acik";          // "Ramp Open"
            currentLine2 = "Kullanilabilir";                 // Clear second line
          } else {
            currentLine1 = "Rampa Kapali";        // "Ramp Closed"
            currentLine2 = "Giris Bekliyor";    // "Awaiting Entry"
          }
          break;

        default:
          currentLine1 = "Sistem Hatasi";       // "System Error"
          currentLine2 = "Yeniden Baslatin";    // "Restart System"
          break;
      }
    }
  }

  // Clear brief message flags if an obstacle appears
  // or if the state changes away from STOPPED (and they were potentially active but interrupted by a new state change).
  if (obstacleDetectedCurrent || (currentState != STOPPED && (showBriefMessageRampaAcildi || showBriefMessageRampaKapandi))) {
      showBriefMessageRampaAcildi = false;
      showBriefMessageRampaKapandi = false;
  }

  // Actual LCD print logic
  if (currentLine1 != prevLine1 || currentLine2 != prevLine2) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(currentLine1);
    lcd.setCursor(0, 1); lcd.print(currentLine2);
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
  // If manual ramp motor was running, ensure it's marked as stopped too
  if (ramp_manualMotorRunning) {
    ramp_manualMotorRunning = false;
    Serial.println(F("MANUAL_RAMP_CTRL: Stopped due to stopAllMotorsAndGoToStopped call."));
  }
  currentState = STOPPED;
  // When stopping due to obstacle or forced stop, clear brief messages
  showBriefMessageRampaAcildi = false;
  showBriefMessageRampaKapandi = false;
  Serial.println(F("SYS_STATE: -> STOPPED (All motors off)"));
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println(F("========================================="));
  Serial.println(F("Enhanced Ramp + Stepper V11 (User LCD + Manual Ramp) Starting..."));
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

  // YENİ EKLENEN: Lineer Aktüatör (Rampa) Manuel Kontrol Pinleri
  pinMode(rampDirButtonPin, INPUT_PULLUP);
  pinMode(rampStartStopButtonPin, INPUT_PULLUP);
  Serial.println(F("- Linear Actuator (Ramp) Manual Control Pins Initialized (Pins 11, 12)."));

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

  updateLCD(); // Initial LCD display
  Serial.println(F("========================================="));
  Serial.println(F("System Ready. Waiting for input..."));
  Serial.println(F("========================================="));
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

// YENİ EKLENEN FONKSİYON: Rampa motoru için manuel kontrol
void handleRampButtonsManual(unsigned long currentTime) {
    static unsigned long lastRampBtnCheckTime_local = 0; // Yerel yoklama zamanlayıcı değişkeni için farklı bir isim
    static bool currentRampDirButtonState_local = HIGH;    // Butonun mantıksal (parazitten arındırılmış) durumu
    static bool currentRampStartStopState_local = HIGH;  // Butonun mantıksal (parazitten arındırılmış) durumu

    // Butonları çok sık kontrol etmemek için (20ms aralık)
    if (currentTime - lastRampBtnCheckTime_local < 20) return;
    lastRampBtnCheckTime_local = currentTime;

    // Çok önemli: Manuel rampa kontrolü sadece sistem 'STOPPED' durumundaysa ve
    // bir engel nedeniyle durmamışsa veya ana sistem tarafından aktif bir işlemde değilse çalışır.
    if (currentState != STOPPED || wasStoppedByObstacle) {
        if (ramp_manualMotorRunning) { // Eğer manuel çalışıyorken sistem durumu değişirse motoru durdur
            analogWrite(RPWM, 0);
            analogWrite(LPWM, 0);
            ramp_manualMotorRunning = false; // Durdu olarak işaretle
            Serial.println(F("MANUAL_RAMP_CTRL: Sistem STOPPED durumundan çıktığı veya engel olduğu için otomatik durduruldu."));
        }
        return; // Sistem meşgulse veya uygun durumda değilse fonksiyondan çık
    }

    // Rampa Başlat/Durdur Butonu Mantığı (Pin 12)
    int startStopReading = digitalRead(rampStartStopButtonPin);
    if (startStopReading != lastRampStartStopButtonState_manual) { // Ham durum değiştiyse parazit önleme zamanlayıcısını sıfırla
        lastRampStartStopDebounceTime_manual = currentTime;
    }
    if ((currentTime - lastRampStartStopDebounceTime_manual) > debounceDelay) {
        // Parazit önleme süresi geçtikten sonra, mantıksal durumun değişip değişmediğini kontrol et
        if (startStopReading != currentRampStartStopState_local) {
            currentRampStartStopState_local = startStopReading; // Mantıksal durumu güncelle
            if (currentRampStartStopState_local == LOW) { // Butona basıldı (LOW)
                ramp_manualMotorRunning = !ramp_manualMotorRunning; // Motorun çalışma durumunu tersine çevir
                Serial.print(F("MANUAL_RAMP_BTN(12): Baslat/Durdur. Rampa Manuel Motor Calismasi: "));
                Serial.println(ramp_manualMotorRunning ? F("ACIK") : F("KAPALI"));

                if (ramp_manualMotorRunning) {
                    if (ramp_manualDirectionForward) { // Yöne göre motoru sür
                        analogWrite(RPWM, motorSpeedRamp);
                        analogWrite(LPWM, 0);
                        Serial.println(F("MANUAL_RAMP_CTRL: ILERI hareket (Aciliyor)"));
                    } else {
                        analogWrite(RPWM, 0);
                        analogWrite(LPWM, motorSpeedRamp);
                        Serial.println(F("MANUAL_RAMP_CTRL: GERI hareket (Kapaniyor)"));
                    }
                } else { // Motor durduruluyorsa
                    analogWrite(RPWM, 0);
                    analogWrite(LPWM, 0);
                    Serial.println(F("MANUAL_RAMP_CTRL: Durduruldu."));
                    // NOT: Manuel operasyon 'rampOpen' bayrağını DEĞİŞTİRMEZ.
                    // Bu, NEMA manuel kontrolüne benzer şekilde, sistemin genel durumunu etkilememesi içindir.
                }
            }
        }
    }
    lastRampStartStopButtonState_manual = startStopReading; // Bir sonraki döngü için ham durumu sakla

    // Rampa Yön Butonu Mantığı (Pin 11)
    int dirReading = digitalRead(rampDirButtonPin);
    if (dirReading != lastRampDirButtonState_manual) { // Ham durum değiştiyse parazit önleme zamanlayıcısını sıfırla
        lastRampDirDebounceTime_manual = currentTime;
    }
    if ((currentTime - lastRampDirDebounceTime_manual) > debounceDelay) {
        // Parazit önleme süresi geçtikten sonra, mantıksal durumun değişip değişmediğini kontrol et
        if (dirReading != currentRampDirButtonState_local) {
            currentRampDirButtonState_local = dirReading; // Mantıksal durumu güncelle
            if (currentRampDirButtonState_local == LOW) { // Butona basıldı (LOW)
                // ÖNEMLİ: Yönü sadece motor manuel olarak çalışmıyorken değiştir
                if (!ramp_manualMotorRunning) {
                    ramp_manualDirectionForward = !ramp_manualDirectionForward; // Yönü tersine çevir
                    Serial.print(F("MANUAL_RAMP_BTN(11): Yon Degistirildi. Rampa Manuel Yonu: "));
                    Serial.println(ramp_manualDirectionForward ? F("ILERI (Acma)") : F("GERI (Kapama)"));
                } else {
                    Serial.println(F("MANUAL_RAMP_BTN(11): Rampa motoru manuel calisirken yon degistirilemez. Once motoru durdurun."));
                }
            }
        }
    }
    lastRampDirButtonState_manual = dirReading; // Bir sonraki döngü için ham durumu sakla
}


void loop() {
  unsigned long now = millis();

  handleStepperButtonsManual(now);
  handleRampButtonsManual(now); // YENİ EKLENEN ÇAĞRI: Rampa manuel kontrolünü işle

  static unsigned long lastIRCheckTime = 0;
  if (now - lastIRCheckTime > 100) {
    int irReading = analogRead(irAnalogPin);
    bool prevObstacleState = obstacleDetectedCurrent;
    obstacleDetectedCurrent = (irReading > irThreshold);

    if (obstacleDetectedCurrent && !prevObstacleState) {
      lastObstacleTimestamp = now;
      Serial.print(F("IR LOGIC: Obstacle DETECTED. Analog: ")); Serial.print(irReading);
      Serial.print(F(" (Threshold: ")); Serial.print(irThreshold); Serial.println(F(")"));
      
      bool isActiveMotorOpState = (currentState == OPENING_RAMP ||
                                   currentState == CLOSING_RAMP ||
                                   currentState == STEPPER_MOVING_FORWARD ||
                                   currentState == STEPPER_MOVING_BACKWARD);
      if (isActiveMotorOpState && !wasStoppedByObstacle) { 
        Serial.print(F("OBSTACLE: Detected during active state: ")); Serial.println(currentState);
        unsigned long elapsedTime = now - actionStartTimestamp;
        unsigned long currentOperationFullDuration = 0;

        if (currentState == OPENING_RAMP || currentState == CLOSING_RAMP) {
            currentOperationFullDuration = runDurationRamp;
        } else { 
            currentOperationFullDuration = runDurationStepper;
        }

        if (elapsedTime < currentOperationFullDuration) {
            remainingOperationTime = currentOperationFullDuration - elapsedTime;
        } else {
            remainingOperationTime = 0; 
        }

        interruptedState = currentState; 
        wasStoppedByObstacle = true;      
        
        Serial.print(F("OBSTACLE: Stored interrupted state: ")); Serial.println(interruptedState);
        Serial.print(F("OBSTACLE: Remaining time: ")); Serial.println(remainingOperationTime);
        
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
    bool isObstacleForLog = (currentIRReadingForLog > irThreshold);
    Serial.print(F("IR STATUS (1s): Value = ")); Serial.print(currentIRReadingForLog);
    if (isObstacleForLog) Serial.println(F(" - Obstacle DETECTED"));
    else Serial.println(F(" - Path clear"));
    lastIRPeriodicLogTime = now;
  }
  
  static unsigned long lastRFIDResetTime = 0;
  if (now - lastRFIDResetTime > 60000) { 
    Serial.println(F("RFID: Performing periodic PCD_Reset and PCD_Init."));
    rfid.PCD_Reset(); delayMicroseconds(100); rfid.PCD_Init(); rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
    lastRFIDResetTime = now; Serial.println(F("RFID: Periodic reset complete."));
  }
  
  static unsigned long lastRFIDCheckTime = 0;
  // RFID okuması sadece sistem 'STOPPED' durumundaysa, engel tarafından durdurulmamışsa
  // ve manuel rampa kontrolü aktif değilse yapılmalı.
  if (currentState == STOPPED && !wasStoppedByObstacle && !ramp_manualMotorRunning) {
    if (!rampOpen && (now - lastRFIDCheckTime > 200)) { 
        if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
            String currentCardUID = getUIDString(rfid.uid);
            Serial.print(F("RFID: New card detected. ")); printUID(rfid.uid);

            if (now - lastSuccessfulRFIDReadTime < rfidReadCooldown && currentCardUID == lastReadCardUID) {
                Serial.println(F("RFID: Same card read within cooldown. Ignoring."));
            } else {
                lastSuccessfulRFIDReadTime = now;
                lastReadCardUID = currentCardUID;

                if (isCardAuthorized(currentCardUID)) {
                    if (!obstacleDetectedCurrent) { 
                        Serial.println(F("RFID: Authorized card. Initiating sequence."));
                        isForwardSequenceInitiatedByRFID = true;
                        currentState = CARD_READ_PROCESSING;
                        actionStartTimestamp = now; // For YETKILI_KART_DISPLAY_TIME and cardProcessingDisplayTime
                        wasStoppedByObstacle = false; 
                        remainingOperationTime = 0;
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
  }

  static unsigned long lastMainButtonCheckTime = 0;
  static bool prevMainButtonState = HIGH;
  if (now - lastMainButtonCheckTime > 50) {
    bool currentMainButtonState = digitalRead(buttonPin);
    if (currentMainButtonState == LOW && prevMainButtonState == HIGH) { // Buton basıldı
      Serial.println(F("MAIN_BTN(10): Pressed."));
      // Clear any brief messages if button is pressed
      showBriefMessageRampaAcildi = false;
      showBriefMessageRampaKapandi = false;

      // Eğer manuel rampa kontrolü aktifse, ana buton onu durdurup sistemi normale döndürsün.
      if (ramp_manualMotorRunning) {
          ramp_manualMotorRunning = false;
          analogWrite(RPWM, 0);
          analogWrite(LPWM, 0);
          Serial.println(F("MANUAL_RAMP_CTRL: Main button pressed, manual ramp control stopped."));
          // currentState zaten STOPPED olmalı, bu yüzden tekrar set etmeye gerek yok.
          // Diğer flag'leri (wasStoppedByObstacle vb.) burada resetlemeye gerek yok,
          // çünkü manuel mod zaten bu flag'ler uygunsa çalışır.
      } else if (currentState == STOPPED) {
        if (wasStoppedByObstacle) { 
          if (!obstacleDetectedCurrent) {
            Serial.println(F("MAIN_BTN(10): Resuming operation after obstacle cleared."));
            currentState = interruptedState; 
            actionStartTimestamp = now;      
            
            if (currentState == STEPPER_MOVING_FORWARD) {
                motorDirection = true;
                digitalWrite(DIR_PIN, HIGH);
                motorRunning = true;
                 Serial.println(F("RESUME_SETUP: Stepper FORWARD, DIR=HIGH, motorRunning=true"));
            } else if (currentState == STEPPER_MOVING_BACKWARD) {
                motorDirection = false;
                digitalWrite(DIR_PIN, LOW);
                motorRunning = true;
                Serial.println(F("RESUME_SETUP: Stepper BACKWARD, DIR=LOW, motorRunning=true"));
            }
            Serial.print(F("SYS_STATE: -> RESUMING to ")); Serial.println(currentState);
            Serial.print(F("Remaining duration: ")); Serial.println(remainingOperationTime);
          } else {
            Serial.println(F("MAIN_BTN(10): Obstacle still present. Cannot resume."));
          }
        } else if (!obstacleDetectedCurrent) { 
          if (!rampOpen) {
            Serial.println(F("MAIN_BTN(10): Initiating FORWARD sequence."));
            isForwardSequenceInitiatedByRFID = false;
            currentState = CARD_READ_PROCESSING;
            actionStartTimestamp = now; // For cardProcessingDisplayTime
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED; 
          } else {
            Serial.println(F("MAIN_BTN(10): Initiating REVERSE sequence."));
            currentState = CLOSING_RAMP;
            actionStartTimestamp = now;
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED; 
          }
        } else { 
          Serial.println(F("MAIN_BTN(10): Pressed, but OBSTACLE. New operation blocked."));
          currentState = INVALID_CARD_DETECTED; 
          actionStartTimestamp = now;
        }
      } else { // Sistem STOPPED değilse (aktif bir işlemde veya hata durumunda)
        stopAllMotorsAndGoToStopped("Main Button (10) pressed during active/error state");
        wasStoppedByObstacle = false; 
        remainingOperationTime = 0;
        interruptedState = STOPPED;
        isForwardSequenceInitiatedByRFID = false;
        rampOpenedByRFIDGlobal = false;
      }
    }
    prevMainButtonState = currentMainButtonState;
    lastMainButtonCheckTime = now;
  }


  switch (currentState) {
    case CARD_READ_PROCESSING:
      // YETKILI_KART_DISPLAY_TIME is handled by updateLCD.
      // This state duration (cardProcessingDisplayTime) is for the underlying logic before motor starts.
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
            Serial.println(F("SYS_STATE: CARD_READ_PROCESSING -> OBSTACLE DETECTED POST-CHECK. Blocking operation."));
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
      { 
        unsigned long durationToUseStepperFwd = wasStoppedByObstacle ? remainingOperationTime : runDurationStepper;
        if (!motorRunning && wasStoppedByObstacle) motorRunning = true; 

        if ((now - actionStartTimestamp) >= durationToUseStepperFwd) {
          Serial.print(F("STEPPER_CTRL: NEMA FORWARD time elapsed. Used duration: ")); Serial.println(durationToUseStepperFwd);
          motorRunning = false;
          digitalWrite(PUL_PIN, LOW);
          
          if (wasStoppedByObstacle) {
            Serial.println(F("STEPPER_CTRL: Resumed NEMA FORWARD completed."));
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }

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
      }
      analogWrite(RPWM, 0); analogWrite(LPWM, 0);
      break;

    case OPENING_RAMP:
      { 
        unsigned long durationToUseRampOpen = wasStoppedByObstacle ? remainingOperationTime : runDurationRamp;
        if ((now - actionStartTimestamp) < durationToUseRampOpen) {
          analogWrite(RPWM, motorSpeedRamp); analogWrite(LPWM, 0);
        } else {
          Serial.print(F("RAMP_CTRL: Ramp OPENED. Used duration: ")); Serial.println(durationToUseRampOpen);
          analogWrite(RPWM, 0); analogWrite(LPWM, 0);
          rampOpen = true;
          rampOpenedByRFIDGlobal = isForwardSequenceInitiatedByRFID; 
          
          if (wasStoppedByObstacle) {
            Serial.println(F("RAMP_CTRL: Resumed RAMP OPENING completed."));
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }
          
          // ----- SET FLAG FOR LCD BRIEF MESSAGE -----
          showBriefMessageRampaAcildi = true; 
          showBriefMessageRampaKapandi = false; 
          briefMessageDisplayUntil = now + BRIEF_MESSAGE_DURATION;
          // ----------------------------------------
          
          currentState = STOPPED;
          Serial.print(F("SYS_STATE: OPENING_RAMP -> STOPPED. Ramp opened by RFID: "));
          Serial.println(rampOpenedByRFIDGlobal ? F("true") : F("false"));
        }
      }
      break;

    case CLOSING_RAMP:
      { 
        unsigned long durationToUseRampClose = wasStoppedByObstacle ? remainingOperationTime : runDurationRamp;
        if ((now - actionStartTimestamp) < durationToUseRampClose) {
          analogWrite(RPWM, 0); analogWrite(LPWM, motorSpeedRamp);
        } else {
          Serial.print(F("RAMP_CTRL: Ramp CLOSED. Used duration: ")); Serial.println(durationToUseRampClose);
          analogWrite(RPWM, 0); analogWrite(LPWM, 0);
          rampOpen = false;
          
          if (wasStoppedByObstacle) {
            Serial.println(F("RAMP_CTRL: Resumed RAMP CLOSING completed."));
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }

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
      }
      break;

    case STEPPER_MOVING_BACKWARD:
      { 
        unsigned long durationToUseStepperBwd = wasStoppedByObstacle ? remainingOperationTime : runDurationStepper;
        if (!motorRunning && wasStoppedByObstacle) motorRunning = true; 

        if ((now - actionStartTimestamp) >= durationToUseStepperBwd) {
          Serial.print(F("STEPPER_CTRL: NEMA REVERSE time elapsed. Used duration: ")); Serial.println(durationToUseStepperBwd);
          motorRunning = false;
          digitalWrite(PUL_PIN, LOW);

          if (wasStoppedByObstacle) {
            Serial.println(F("STEPPER_CTRL: Resumed NEMA BACKWARD completed."));
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }
          
          // ----- SET FLAG FOR LCD BRIEF MESSAGE -----
          showBriefMessageRampaKapandi = true; 
          showBriefMessageRampaAcildi = false; 
          briefMessageDisplayUntil = now + BRIEF_MESSAGE_DURATION;
          // ----------------------------------------

          currentState = STOPPED;
          isForwardSequenceInitiatedByRFID = false; 
          rampOpenedByRFIDGlobal = false; 
          Serial.println(F("SYS_STATE: STEPPER_MOVING_BACKWARD -> STOPPED (Reverse Sequence Complete)"));
        }
      }
      analogWrite(RPWM, 0); analogWrite(LPWM, 0); 
      break;

    case STOPPED:
      // Eğer manuel rampa kontrolü aktif değilse, normalde motorlar duruk olmalı.
      // Manuel kontrol kendi motor çıkışlarını yönetir.
      if (!ramp_manualMotorRunning) {
        analogWrite(RPWM, 0); 
        analogWrite(LPWM, 0);
      }
      // Diğer motorlar (NEMA) zaten STOPPED durumunda durmuş olmalı.
      break;
    default:
      break;
  }

  static unsigned long lastLCDUpdateTime = 0;
  if (now - lastLCDUpdateTime > 250) { 
    updateLCD();
    lastLCDUpdateTime = now;
  }
  
  static unsigned long lastStatusReportTime = 0;
  if (now - lastStatusReportTime > 60000) { 
    Serial.println(F("--- Status Report ---"));
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
    Serial.print(F("  Was Stopped By Obstacle: ")); Serial.println(wasStoppedByObstacle ? "Yes" : "No");
    if(wasStoppedByObstacle){
      // Serial.print(F("  Interrupted State: ")); Serial.println(interruptedState); // Need a helper to print enum name
      Serial.print(F("  Remaining Operation Time: ")); Serial.println(remainingOperationTime);
    }
    Serial.print(F("  NEMA Motor Running (Auto): ")); Serial.println(motorRunning ? "Yes" : "No"); // Clarify this is auto NEMA
    Serial.print(F("  NEMA Motor Direction (true=fwd): ")); Serial.println(motorDirection);
    Serial.print(F("  Manual Ramp Motor Running: ")); Serial.println(ramp_manualMotorRunning ? "Yes" : "No");
    Serial.print(F("  Manual Ramp Direction (true=fwd/open): ")); Serial.println(ramp_manualDirectionForward);
    Serial.print(F("  BriefMsg Acildi: ")); Serial.print(showBriefMessageRampaAcildi);
    Serial.print(F(" Kapandi: ")); Serial.print(showBriefMessageRampaKapandi);
    Serial.print(F(" Until: ")); Serial.println(briefMessageDisplayUntil);
    Serial.println(F("---------------------------"));
    lastStatusReportTime = now;
  }
}