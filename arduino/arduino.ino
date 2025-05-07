#include <SPI.h>
#include <MFRC522.h>

// RFID Modulu Pinleri
#define SS_PIN 53
#define RST_PIN 49
MFRC522 rfid(SS_PIN, RST_PIN);

// Pin Tanimlari
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

  Serial.println("Sistem hazir!");
}

void loop() {
  bool pirDetected   = digitalRead(pirPin) == HIGH;
  bool buttonPressed = digitalRead(buttonPin) == LOW;

  // --- RFID ile rampayi acma ---
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("Kart algilandi: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    if (!rampOpen) {
      if (!pirDetected) {
        rampaAc();
      } else {
        Serial.println("Engel algilandi! Rampa acilamiyor.");
      }
    } else {
      Serial.println("Rampa zaten acik.");
    }

    rfid.PICC_HaltA();
  }

  // --- Buton ile ac/kapat ---
  if (buttonPressed) {
    delay(50);  // debounce
    while (digitalRead(buttonPin) == LOW) delay(10);

    if (!rampOpen) {
      Serial.println("Buton: Rampa aciliyor...");
      if (!pirDetected) {
        rampaAc();
      } else {
        Serial.println("Engel algilandi! Rampa acilamiyor.");
      }
    } else {
      Serial.println("Buton: Rampa kapaniyor...");
      rampaKapat();
    }
  }
}

// Rampa acma islemi (ileri hareket)
void rampaAc() {
  Serial.println("Rampa aciliyor...");
  rampOpen = true;
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 200);
  delay(16000);  // Aktuator acilma suresi
  motorDur();
  Serial.println("Rampa acik durumda.");
}

// Rampa kapatma islemi (geri hareket)
void rampaKapat() {
  Serial.println("Rampa kapaniyor...");
  analogWrite(RPWM, 200);
  analogWrite(LPWM, 0);
  delay(16000);  // Aktuator kapanma suresi
  motorDur();
  rampOpen = false;
  Serial.println("Rampa kapali.");
}

// Motoru durdur
void motorDur() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}