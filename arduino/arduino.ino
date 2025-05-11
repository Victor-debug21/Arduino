#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// I2C LCD (address 0x27, 16 cols x 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID module pins
#define SS_PIN   53
#define RST_PIN  49
MFRC522 rfid(SS_PIN, RST_PIN);

// Pin definitions
const int buttonPin       = 3;               // Open/close toggle button
const int pirPin          = 4;               // PIR sensor (optional)
const int irAnalogPins[4] = {A0, A1, A2, A3};// IR sensor analog inputs
const int RPWM            = 5;               // Motor forward PWM
const int LPWM            = 6;               // Motor reverse PWM
const int REN             = 7;               // Motor driver enable
const int LEN             = 8;               // Motor driver enable

// Thresholds / timing
const int irThreshold         = 200;         // IR trip threshold (~2 cm)
const unsigned long runDuration = 16000;     // ms: full travel time
const int motorSpeed          = 200;         // PWM speed (0–255)
const unsigned long obstacleTimeout = 5000;  // 5 seconds timeout after obstacle

// Ramp state machine
enum RampState { STOPPED, OPENING, CLOSING };
RampState rampState = STOPPED;
bool    rampOpen    = false;
unsigned long actionStart = 0;
unsigned long lastObstacleTime = 0;
bool obstacleDetected = false;

// Helper: print to Serial and LCD with better formatting
void logMsg(const String &line1, const String &line2 = "") {
  Serial.println(line1);
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print(line1);
  if (line2.length()) {
    lcd.setCursor(0,1); 
    lcd.print(line2);
  }
}



// Display ramp status on LCD
void updateLCD() {
  String line1, line2;
  
  switch (rampState) {
    case OPENING:
      line1 = "Rampa Aciliyor";
      line2 = "Lutfen Bekleyin";
      break;
    case CLOSING:
      line1 = "Rampa Kapaniyor";
      line2 = "Lutfen Bekleyin";
      break;
    case STOPPED:
      if (obstacleDetected) {
        line1 = "Engel Algilandi!";
        line2 = "Yol Temizleyin";
      } else if (rampOpen) {
        line1 = "Rampa Acik";
        line2 = "Kullanilabilir";
      } else {
        line1 = "Rampa Kapali";
        line2 = "Kart Okutun";
      }
      break;
  }
  
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(line1);
  lcd.setCursor(0,1); lcd.print(line2);
}

void setup() {
  Serial.begin(9600);
  while (!Serial); // For Mega: Wait for serial port to connect
  Serial.println(F("Rampa Kontrol Sistemi Basliyor..."));
  
  SPI.begin();
  
  // RFID init with retries
  byte maxRetries = 5;
  bool rfidInitialized = false;
  
  for (byte i = 0; i < maxRetries && !rfidInitialized; i++) {
    rfid.PCD_Init();
    delay(50);  // Give some time for the RFID module to initialize
    rfidInitialized = rfid.PCD_PerformSelfTest();
    if (!rfidInitialized) {
      Serial.println(F("RFID başlatma denemesi başarısız, tekrar deneniyor..."));
      delay(200);
    }
  }
  
  if (rfidInitialized) {
    Serial.println(F("RFID modülü başarıyla başlatıldı."));
    // Increase RFID antenna gain for better range
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
    rfid.PCD_DumpVersionToSerial(); // Show RFID reader details
  } else {
    Serial.println(F("RFID başlatılamadı! Kontrol ediniz."));
  }

  // LCD init
  Wire.begin();
  lcd.init();
  lcd.backlight();
  logMsg("Sistem Hazir", "Kart Bekliyor");

  // Pin modes
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pirPin,    INPUT);
  for (int i = 0; i < 4; i++) pinMode(irAnalogPins[i], INPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN,  OUTPUT);
  pinMode(LEN,  OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  
  // Initial LCD update
  updateLCD();
}

void loop() {
  unsigned long now = millis();
  bool pirDetected = (digitalRead(pirPin) == HIGH);
  bool irDetected  = anyIRDetected();
  obstacleDetected = pirDetected || irDetected;

  // Check if obstacle timeout has passed
  if (obstacleDetected) {
    lastObstacleTime = now;
  }
  
  bool canOperate = (now - lastObstacleTime >= obstacleTimeout);
  
  // Regular RFID reset to prevent lockups
  static unsigned long lastRFIDReset = 0;
  if (now - lastRFIDReset > 10000) { // Reset RFID reader every 10 seconds
    rfid.PCD_Reset();
    delay(50); // Small delay after reset
    rfid.PCD_Init();
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max); // Restore gain setting
    lastRFIDReset = now;
  }
  
  // --- Improved RFID detection ---
  if (rfid.PICC_IsNewCardPresent()) {
    if (rfid.PICC_ReadCardSerial()) {
      // Print UID to serial
      Serial.print("Kart algilandi: ");
      for (byte i = 0; i < rfid.uid.size; i++) {
        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      // Accept any card
        if (rampState == STOPPED) {
          if (!obstacleDetected || canOperate) {
            if (!rampOpen) {
              rampState   = OPENING;
              actionStart = now;
              logMsg("Kart Dogrulandi", "Rampa Aciliyor");
            } else {
              rampState   = CLOSING;
              actionStart = now;
              logMsg("Kart Dogrulandi", "Rampa Kapaniyor");
            }
          } else {
            logMsg("Engel Var!", "5sn Bekleyin");
          }
        } else {
          logMsg("Rampa Harekette", "Lutfen Bekleyin");
        }

      
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  }

  // --- Button control (same logic) ---
  if (digitalRead(buttonPin) == LOW) {
    delay(50); // Debounce
    while (digitalRead(buttonPin) == LOW); // Wait for release
    
    if (rampState == STOPPED) {
      if (!obstacleDetected || canOperate) {
        if (!rampOpen) {
          rampState   = OPENING;
          actionStart = now;
          logMsg("Buton Basildi", "Rampa Aciliyor");
        } else {
          rampState   = CLOSING;
          actionStart = now;
          logMsg("Buton Basildi", "Rampa Kapaniyor");
        }
      } else {
        logMsg("Engel Var!", "5sn Bekleyin");
      }
    } else {
      logMsg("Rampa Harekette", "Lutfen Bekleyin");
    }
  }

  // --- State machine for motor control ---
  switch (rampState) {
    case OPENING:
      if ((now - actionStart) < runDuration && !anyIRDetected()) {
        // Forward motion
        analogWrite(RPWM, 0);
        analogWrite(LPWM, motorSpeed);
      } else {
        motorStop();
        rampOpen = true;
        if (anyIRDetected()) {
          logMsg("Engel Algilandi!", "Hareket Durdu");
        } else {
          logMsg("Rampa Acik", "Kullanilabilir");
        }
        rampState = STOPPED;
      }
      break;

    case CLOSING:
      if ((now - actionStart) < runDuration && !anyIRDetected()) {
        // Reverse motion
        analogWrite(RPWM, motorSpeed);
        analogWrite(LPWM, 0);
      } else {
        motorStop();
        rampOpen = false;
        if (anyIRDetected()) {
          logMsg("Engel Algilandi!", "Hareket Durdu");
        } else {
          logMsg("Rampa Kapali", "Kart Bekliyor");
        }
        rampState = STOPPED;
      }
      break;

    case STOPPED:
    default:
      // Motor off
      motorStop();
      break;
  }
  
  // Update LCD every 500ms when stopped to avoid flickering during motion
  static unsigned long lastLCDUpdate = 0;
  if (rampState == STOPPED && (now - lastLCDUpdate > 500)) {
    updateLCD();
    lastLCDUpdate = now;
  }
  
  // Check and display IR sensors values periodically
  static unsigned long lastIRCheck = 0;
  if (now - lastIRCheck > 2000) { // Check every 2 seconds to reduce serial traffic
    // Print IR values for monitoring/calibration
    Serial.print("IR Degerler: ");
    for (int i = 0; i < 4; i++) {
      int reading = analogRead(irAnalogPins[i]);
      Serial.print(reading);
      Serial.print(reading < irThreshold ? "(E) " : " "); // Show if value indicates obstacle
    }
    Serial.println();
    lastIRCheck = now;
  }
}

// IR sensor detection with improved reliability
bool anyIRDetected() {
  // Check each IR sensor for obstacles
  for (int i = 0; i < 4; i++) {
    // Take 3 readings to confirm detection and avoid false positives
    int belowThresholdCount = 0;
    
    for (int j = 0; j < 3; j++) {
      int reading = analogRead(irAnalogPins[i]);
      if (reading < irThreshold) {
        belowThresholdCount++;
      }
      // Small delay between readings for stability
      delayMicroseconds(500);
    }
    
    // If at least 2 out of 3 readings detect an obstacle (below threshold)
    // then consider it a valid detection for this sensor
    if (belowThresholdCount >= 2) {
      return true; // Obstacle detected
    }
  }
  
  // No obstacles detected on any sensor
  return false;
}

// Stop motor
void motorStop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}