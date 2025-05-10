#include <SPI.h>
#include <MFRC522.h>

// --- RFID Modülü Pinleri ---
#define SS_PIN   53
#define RST_PIN  49
MFRC522 rfid(SS_PIN, RST_PIN);

// --- Pin Tanımları ---
const int buttonPin       = 3;               // Butonla aç/kapat
const int pirPin          = 4;               // PIR sensör (isteğe bağlı)
const int irAnalogPins[4] = {A0, A1, A2, A3}; // IR sensörlerin AO pinleri
const int RPWM            = 5;               // Motor sağ yönde PWM
const int LPWM            = 6;               // Motor sol yönde PWM
const int REN             = 7;               // Motor sürücü enable
const int LEN             = 8;               // Motor sürücü enable

bool rampOpen = false;
// Yaklaşık 2 cm için analog eşik (0–1023 arası)
// Ölçümlerinize göre ~200 civarı değeri test edin
const int irThreshold = 200;
// Aktüatör çalışma süresi (ms)
const unsigned long runDuration = 16000;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pirPin,    INPUT);
  for (int i = 0; i < 4; i++) pinMode(irAnalogPins[i], INPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN,  OUTPUT);
  pinMode(LEN,  OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  Serial.println("Sistem hazir! IR thresh=200 (~2cm).");
}

void loop() {
  bool pirDetected = (digitalRead(pirPin) == HIGH);
  bool irDetected  = anyIRDetected();
  bool obstacle    = pirDetected || irDetected;

  // RFID ile açma
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print("Kart algilandi: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    if (!rampOpen) {
      if (!obstacle) rampaAc();
      else           Serial.println("Engel var, acilmiyor.");
    } else {
      Serial.println("Rampa zaten acik.");
    }
    rfid.PCD_StopCrypto1();
  }

  // Buton ile aç/kapat
  if (digitalRead(buttonPin) == LOW) {
    delay(50);
    while (digitalRead(buttonPin) == LOW) delay(10);

    if (!rampOpen) {
      if (!obstacle) rampaAc();
      else           Serial.println("Engel var, acilmiyor.");
    } else {
      if (!obstacle) rampaKapat();
      else           Serial.println("Engel var, kapatılamiyor.");
    }
  }
}

// IR sensörlerinden herhangi biri eşiğin altına inerse true
bool anyIRDetected() {
  for (int i = 0; i < 4; i++) {
    int v = analogRead(irAnalogPins[i]);
    if (v < irThreshold) return true;
  }
  return false;
}

// Aktüatör kontrolü: süre boyunca her döngüde IR kontrolü yapar
void rampaAc() {
  Serial.println("Rampa aciliyor...");
  rampOpen = true;
  unsigned long start = millis();
  // İleri yönde çalıştır
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 200);

  while (millis() - start < runDuration) {
    if (anyIRDetected()) {
      Serial.println("Engel algilandi! Hareket durduruldu.");
      break;
    }
    // RFID okumaları arkaplanda çalışabilir
    if (rfid.PICC_IsNewCardPresent()) rfid.PCD_StopCrypto1();
  }
  motorDur();
  Serial.println("Rampa hareketi sonlandi.");
}

// Aktüatör geri yönde kontrolü
void rampaKapat() {
  Serial.println("Rampa kapaniyor...");
  unsigned long start = millis();
  analogWrite(RPWM, 200);
  analogWrite(LPWM, 0);

  while (millis() - start < runDuration) {
    if (anyIRDetected()) {
      Serial.println("Engel algilandi! Hareket durduruldu.");
      break;
    }
    if (rfid.PICC_IsNewCardPresent()) rfid.PCD_StopCrypto1();
  }
  motorDur();
  rampOpen = false;
  Serial.println("Rampa kapatma sonlandi.");
}

// Motoru durdur
void motorDur() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}