#include <SPI.h>
#include <MFRC522.h>

// RFID Modülü Pinleri
#define SS_PIN 53
#define RST_PIN 49
MFRC522 rfid(SS_PIN, RST_PIN);

// Pin Tanımları
const int buttonPin = 3;
const int pirPin    = 4;
const int RPWM      = 5;
const int LPWM      = 6;
const int REN       = 7;
const int LEN       = 8;

bool rampOpen = false;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pirPin, INPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  Serial.println("Sistem Hazır!");
}

void loop() {
  bool pirDetected   = digitalRead(pirPin) == HIGH;
  bool buttonPressed = digitalRead(buttonPin) == LOW;

  // --- RFID ile yetki kontrolü olmadan açma ---
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("Kart algılandı: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    if (!rampOpen) {
      if (!pirDetected) {
        openRamp();
      } else {
        Serial.println("Engel algılandı! Rampa açılamıyor.");
      }
    } else {
      Serial.println("Rampa zaten açık.");
    }

    rfid.PICC_HaltA();
  }

  // --- Buton ile açma/kapatma toggle ---
  if (buttonPressed) {
    // Debounce için kısa bekleme
    delay(50);
    while (digitalRead(buttonPin) == LOW) delay(10);

    if (!rampOpen) {
      Serial.println("Buton: Açılıyor...");
      if (!pirDetected) {
        openRamp();
      } else {
        Serial.println("Engel algılandı! Rampa açılamıyor.");
      }
    } else {
      Serial.println("Buton: Kapanıyor...");
      closeRamp();
    }
  }
}

void openRamp() {
  Serial.println("Rampa açılıyor...");
  rampOpen = true;
  analogWrite(RPWM, 200);
  analogWrite(LPWM, 0);
  delay(7000);    // Aktüatör açılma süresi
  stopRamp();
  Serial.println("Rampa açık durumda.");
}

void closeRamp() {
  Serial.println("Rampa kapanıyor...");
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 200);
  delay(7000);    // Aktüatör kapanma süresi
  stopRamp();
  rampOpen = false;
  Serial.println("Rampa kapandı.");
}

void stopRamp() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}
