#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// I2C LCD (adres 0x27, 16 sütun x 2 satır)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID modül pinleri
#define SS_PIN   53
#define RST_PIN  49
MFRC522 rfid(SS_PIN, RST_PIN);

// Pin tanımlamaları
const int buttonPin     = 3;     // Açma/kapama düğmesi
const int irAnalogPin   = A0;    // Tek IR sensör analog girişi (sadece bir tane kullanılacak)
const int RPWM          = 5;     // Motor ileri PWM
const int LPWM          = 6;     // Motor geri PWM
const int REN           = 7;     // Motor sürücü etkinleştirme
const int LEN           = 8;     // Motor sürücü etkinleştirme

// Eşikler / zamanlamalar
const int irThreshold          = 200;     // IR tetikleme eşiği (~2 cm)
const unsigned long runDuration = 16000;  // ms: tam hareket süresi
const int motorSpeed           = 200;     // PWM hızı (0-255)
const unsigned long obstacleTimeout = 5000; // Engel sonrası 5 saniye bekleme süresi

// Rampa durum makinesi
enum RampState { STOPPED, OPENING, CLOSING };
RampState rampState = STOPPED;
bool rampOpen = false;
unsigned long actionStart = 0;
unsigned long lastObstacleTime = 0;
bool obstacleDetected = false;

// Yardımcı: Seri port ve LCD'ye daha iyi formatlama ile yazdırma
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

// LCD'de rampa durumunu göster
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
  while (!Serial); // Mega için: Seri bağlantı kurulana kadar bekle
  Serial.println(F("Rampa Kontrol Sistemi Basliyor..."));
  
  SPI.begin();
  
  // Pin modları
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(irAnalogPin, INPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN,  OUTPUT);
  pinMode(LEN,  OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);
  
  // LCD başlatma
  Wire.begin();
  lcd.init();
  lcd.backlight();
  logMsg("Sistem Baslatiliyor", "Lutfen Bekleyin");
  
  // RFID başlatma - optimizasyon yapıldı
  delay(1000); // Sistem başlarken biraz bekleyelim
  rfid.PCD_Init();
  delay(100);  // RFID modülünün başlaması için biraz zaman verelim
  
  // Baslatma durumunu kontrol etme
  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  if (version == 0x00 || version == 0xFF) {
    Serial.println(F("RFID modulu bulunamadi veya yanlis baglanti"));
  } else {
    Serial.println(F("RFID modulu baglandi"));
    // Daha iyi menzil için RFID anten kazancını artır
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
    rfid.PCD_DumpVersionToSerial(); // RFID okuyucu detaylarını göster
  }

  // İlk LCD güncelleme
  logMsg("Sistem Hazir", "Kart Bekliyor");
  updateLCD();
}

void loop() {
  unsigned long now = millis();
  bool irDetected = isIRDetected();
  obstacleDetected = irDetected;

  // Engel zaman aşımının geçip geçmediğini kontrol et
  if (obstacleDetected) {
    lastObstacleTime = now;
  }
  
  bool canOperate = (now - lastObstacleTime >= obstacleTimeout);
  
  // Düzenli RFID sıfırlaması
  static unsigned long lastRFIDReset = 0;
  if (now - lastRFIDReset > 10000) { // Her 10 saniyede bir RFID okuyucuyu sıfırla
    rfid.PCD_Reset();
    delay(50); // Sıfırlama sonrası küçük bir gecikme
    rfid.PCD_Init();
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max); // Kazanç ayarını geri yükle
    lastRFIDReset = now;
  }
  
  // --- İyileştirilmiş RFID algılama ---
  if (rfid.PICC_IsNewCardPresent()) {
    if (rfid.PICC_ReadCardSerial()) {
      // UID'yi seriye yazdır
      Serial.print("Kart algilandi: ");
      for (byte i = 0; i < rfid.uid.size; i++) {
        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      // Herhangi bir kartı kabul et
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

  // --- Düğme kontrolü ---
  if (digitalRead(buttonPin) == LOW) {
    delay(50); // Debounce
    while (digitalRead(buttonPin) == LOW); // Bırakılmasını bekle
    
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

  // --- Motor kontrolü için durum makinesi ---
  switch (rampState) {
    case OPENING:
      if ((now - actionStart) < runDuration && !isIRDetected()) {
        // İleri hareket
        analogWrite(RPWM, 0);
        analogWrite(LPWM, motorSpeed);
      } else {
        motorStop();
        rampOpen = true;
        if (isIRDetected()) {
          logMsg("Engel Algilandi!", "Hareket Durdu");
        } else {
          logMsg("Rampa Acik", "Kullanilabilir");
        }
        rampState = STOPPED;
      }
      break;

    case CLOSING:
      if ((now - actionStart) < runDuration && !isIRDetected()) {
        // Geri hareket
        analogWrite(RPWM, motorSpeed);
        analogWrite(LPWM, 0);
      } else {
        motorStop();
        rampOpen = false;
        if (isIRDetected()) {
          logMsg("Engel Algilandi!", "Hareket Durdu");
        } else {
          logMsg("Rampa Kapali", "Kart Bekliyor");
        }
        rampState = STOPPED;
      }
      break;

    case STOPPED:
    default:
      // Motor kapalı
      motorStop();
      break;
  }
  
  // Durduğunda titreşimi önlemek için LCD'yi her 500ms'de bir güncelle
  static unsigned long lastLCDUpdate = 0;
  if (rampState == STOPPED && (now - lastLCDUpdate > 500)) {
    updateLCD();
    lastLCDUpdate = now;
  }
  
  // IR sensör değerini periyodik olarak kontrol et ve göster
  static unsigned long lastIRCheck = 0;
  if (now - lastIRCheck > 2000) { // Seri trafiği azaltmak için her 2 saniyede bir kontrol et
    // IR değerini izleme/kalibrasyon için yazdır
    int reading = analogRead(irAnalogPin);
    Serial.print("IR Deger: ");
    Serial.print(reading);
    Serial.println(reading < irThreshold ? " (Engel var)" : " (Engel yok)");
    lastIRCheck = now;
  }
}

// IR sensör algılaması - geliştirilmiş güvenilirlik
bool isIRDetected() {
  // 3 okuma yap ve algılamayı doğrula (yanlış pozitifleri önle)
  int belowThresholdCount = 0;
  
  for (int j = 0; j < 3; j++) {
    int reading = analogRead(irAnalogPin);
    if (reading < irThreshold) {
      belowThresholdCount++;
    }
    // Kararlılık için okumalar arasında küçük bir gecikme
    delayMicroseconds(500);
  }
  
  // 3 okumanın en az 2'si engel algıladıysa (eşiğin altında)
  // bunu geçerli bir algılama olarak kabul et
  return (belowThresholdCount >= 2);
}

// Motoru durdur
void motorStop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}