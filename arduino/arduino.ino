#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>

// I2C LCD (adres 0x27, 16 sütun x 2 satır)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID modül pinleri
#define SS_PIN 53  // RFID SPI Slave Select pini
#define RST_PIN 49 // RFID Reset pini
MFRC522 rfid(SS_PIN, RST_PIN); // MFRC522 kütüphanesi için nesne oluşturma

// Pin tanımlamaları
const int buttonPin = 10;     // Ana kontrol butonu pini
const int irAnalogPin = A0;   // IR engel sensörü analog giriş pini
const int RPWM = 5;           // Rampa motoru İLERİ PWM pini
const int LPWM = 6;           // Rampa motoru GERİ PWM pini
const int REN = 7;            // Rampa motoru İLERİ yönü etkinleştirme pini
const int LEN = 8;            // Rampa motoru GERİ yönü etkinleştirme pini

// Step Motor pinleri (NEMA 17)
#define DIR_PIN 2        // Step motor yön (Direction) pini
#define PUL_PIN 4        // Step motor pals (Pulse) pini
#define DIR_BUTTON_PIN 3 // Step motor manuel yön değiştirme butonu pini
#define START_STOP_PIN 9 // Step motor manuel başlatma/durdurma butonu pini

// Lineer Aktüatör (Rampa) Manuel Kontrol Pinleri
const int rampDirButtonPin = 11;     // Lineer Aktüatör (Rampa) Manuel Yön Butonu pini
const int rampStartStopButtonPin = 12; // Lineer Aktüatör (Rampa) Manuel Başlat/Durdur Butonu pini

// Step motor değişkenleri - interrupt (kesme) için 'volatile' anahtar kelimesi kullanıldı
volatile bool motorDirection = true;  // Step motor yönü: true = İLERİ, false = GERİ
volatile bool motorRunning = false;   // Step motor çalışıyor mu? (Otomatik mod için)
volatile bool pulseState = false;     // Step motor pals durumu (HIGH/LOW)

// Step motor manuel kontrol buton değişkenleri
bool lastDirButtonState = HIGH;             // Step motor yön butonu son durumu (debounce için)
unsigned long lastDirDebounceTime = 0;    // Step motor yön butonu son basılma zamanı (debounce için)
bool lastStartStopState = HIGH;           // Step motor başlat/durdur butonu son durumu (debounce için)
unsigned long lastStartStopDebounceTime = 0; // Step motor başlat/durdur butonu son basılma zamanı (debounce için)
unsigned long debounceDelay = 50;         // Buton arkı önleme gecikmesi (milisaniye)

// Rampa manuel kontrol buton durum takibi (parazit önleme için)
bool lastRampDirButtonState_manual = HIGH;             // Rampa yön butonu (manuel) son durumu
unsigned long lastRampDirDebounceTime_manual = 0;    // Rampa yön butonu (manuel) son basılma zamanı
bool lastRampStartStopButtonState_manual = HIGH;           // Rampa başlat/durdur butonu (manuel) son durumu
unsigned long lastRampStartStopDebounceTime_manual = 0; // Rampa başlat/durdur butonu (manuel) son basılma zamanı

// Rampa manuel kontrol motor durum değişkenleri
bool ramp_manualMotorRunning = false;     // Rampa motoru manuel olarak çalışıyor mu?
bool ramp_manualDirectionForward = true; // Rampa manuel yönü: true = İLERİ (Açma), false = GERİ (Kapama)


// Sistem değişkenleri
const int irThreshold = 150;              // IR sensörü engel algılama eşik değeri <<< !!! BU DEĞERİ KALİBRE EDİN !!!
const unsigned long runDurationRamp = 2500;       // Rampa motorunun bir yönde çalışma süresi (ms)
const unsigned long runDurationStepper = 14000;     // Step motorun bir yönde çalışma süresi (ms)
const int motorSpeedRamp = 200;             // Rampa motor hızı (PWM değeri, 0-255)
const unsigned long obstacleTimeout = 5000;       // Engel algılandığında bekleme süresi (aktif kullanılmıyor)
const unsigned long rfidReadCooldown = 2000;      // Aynı RFID kartının tekrar okunması için bekleme süresi (ms)
const unsigned long cardProcessingDisplayTime = 1500; // "Kart Okundu" mesajının gösterim süresi
const unsigned long invalidCardDisplayTime = 2000;  // "Geçersiz Kart" mesajının gösterim süresi (ms)

// RFID Yetkilendirme Listesi
String authorizedUids[] = { // Yetkili RFID kart UID'leri
  "D32822DA",
  "11223344"
  // Diğer yetkili UID'lerinizi buraya ekleyebilirsiniz
};
const int numAuthorizedUids = sizeof(authorizedUids) / sizeof(authorizedUids[0]); // Yetkili UID sayısı

enum SystemState { // Sistem durumlarını tanımlayan enum yapısı
  STOPPED,                // Durdu / Beklemede
  CARD_READ_PROCESSING,   // Kart okundu, işlem yapılıyor
  INVALID_CARD_DETECTED,  // Geçersiz kart algılandı
  STEPPER_MOVING_FORWARD, // Step motor ileri hareket ediyor
  OPENING_RAMP,           // Rampa açılıyor
  CLOSING_RAMP,           // Rampa kapanıyor
  STEPPER_MOVING_BACKWARD // Step motor geri hareket ediyor
};
SystemState currentState = STOPPED; // Mevcut sistem durumu, başlangıçta STOPPED

bool rampOpen = false;                      // Rampa açık mı? (Fiziksel durumu)
bool rampOpenedByRFIDGlobal = false;        // Rampa RFID ile mi açıldı? (Global bayrak)
bool isForwardSequenceInitiatedByRFID = false; // İleri yönlü işlemi RFID mi başlattı?
unsigned long actionStartTimestamp = 0;     // Mevcut işlemin başlama zaman damgası
unsigned long lastObstacleTimestamp = 0;    // En son engel algılanma zaman damgası
bool obstacleDetectedCurrent = false;       // Şu anda engel algılanıyor mu?
unsigned long lastSuccessfulRFIDReadTime = 0; // Başarılı son RFID okuma zaman damgası
String lastReadCardUID = "";                // Son okunan RFID kartının UID'si

// Zamanlayıcı ve devam etme fonksiyonları için değişkenler
unsigned long remainingOperationTime = 0; // Engel sonrası kalan işlem süresi
SystemState interruptedState = STOPPED;     // Engel nedeniyle kesintiye uğrayan durum
volatile bool wasStoppedByObstacle = false; // Sistem bir engel tarafından mı durduruldu?

// Kullanıcı Dostu LCD Mesajları İçin Sabitler ve Bayraklar
const unsigned long YETKILI_KART_DISPLAY_TIME = 2000; // "Yetkili Kart" mesajının gösterim süresi (2 saniye)
const unsigned long BRIEF_MESSAGE_DURATION = 2000;    // "Rampa Açıldı/Kapandı" gibi kısa mesajların gösterim süresi (2 saniye)

bool showBriefMessageRampaAcildi = false;  // "Rampa Açıldı" kısa mesajı gösterilsin mi?
bool showBriefMessageRampaKapandi = false; // "Rampa Kapandı" kısa mesajı gösterilsin mi?
unsigned long briefMessageDisplayUntil = 0; // Kısa mesajın ne zamana kadar gösterileceği


void stepperTimerISR() { // Step motor için Timer1 kesme servis rutini
  if (motorRunning) {    // Eğer step motor (otomatik modda) çalışıyorsa
    if (pulseState) {
      digitalWrite(PUL_PIN, HIGH); // Pals pinini HIGH yap
      pulseState = false;            // Pals durumunu ters çevir
    } else {
      digitalWrite(PUL_PIN, LOW);  // Pals pinini LOW yap
      pulseState = true;             // Pals durumunu ters çevir
    }
  } else {
    digitalWrite(PUL_PIN, LOW); // Motor çalışmıyorsa pals pinini LOW tut
  }
}

void printUID(MFRC522::Uid uid) { // Okunan RFID UID'sini seri porta yazdırma fonksiyonu
  Serial.print(F("Kart UID: "));
  for (byte i = 0; i < uid.size; i++) {
    Serial.print(uid.uidByte[i] < 0x10 ? " 0" : " "); // Başına 0 ekle (eğer tek haneli hex ise)
    Serial.print(uid.uidByte[i], HEX); // UID byte'ını HEX formatında yazdır
  }
  Serial.println();
}

String getUIDString(MFRC522::Uid uid) { // Okunan RFID UID'sini String olarak döndüren fonksiyon
  String uidStr = "";
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) uidStr += "0"; // Başına 0 ekle (eğer tek haneli hex ise)
    uidStr += String(uid.uidByte[i], HEX);    // UID byte'ını HEX formatında string'e ekle
  }
  uidStr.toUpperCase(); // String'i büyük harfe çevir
  return uidStr;
}

bool isCardAuthorized(String uid) { // Verilen UID'nin yetkili olup olmadığını kontrol eden fonksiyon
  for (int i = 0; i < numAuthorizedUids; i++) {
    if (uid == authorizedUids[i]) {
      return true; // Yetkili listede bulundu
    }
  }
  return false; // Yetkili listede bulunamadı
}

void updateLCD() { // LCD ekranı güncelleyen ana fonksiyon
  static String prevLine1 = ""; // LCD 1. satırın bir önceki değeri
  static String prevLine2 = ""; // LCD 2. satırın bir önceki değeri
  String currentLine1 = "", currentLine2 = ""; // LCD için mevcut satır içerikleri
  unsigned long now = millis(); // Mevcut zamanı al

  // Kısa Süreli Zamanlanmış Mesajlar (Öncelik 1)
  if (showBriefMessageRampaAcildi && now < briefMessageDisplayUntil && !obstacleDetectedCurrent) {
    currentLine1 = "Rampa Acildi";
    currentLine2 = "Kullanilabilir";
  } else if (showBriefMessageRampaKapandi && now < briefMessageDisplayUntil && !obstacleDetectedCurrent) {
    currentLine1 = "Rampa Kapandi";
    currentLine2 = "Giris Bekliyor";
  } else {
    // Kısa mesaj bayraklarını süreleri dolduysa veya bir engel ortaya çıkarsa sıfırla
    if (now >= briefMessageDisplayUntil || obstacleDetectedCurrent) {
      showBriefMessageRampaAcildi = false;
      showBriefMessageRampaKapandi = false;
    }

    // Engel Mesajları (Öncelik 2)
    if (obstacleDetectedCurrent) {
      currentLine1 = "Engel Algilandi!";
      if (currentState == STOPPED && wasStoppedByObstacle && !obstacleDetectedCurrent) {
        currentLine2 = "Devam icin Buton"; // Engel temizlendi, buton bekleniyor
      } else if (currentState == STOPPED && wasStoppedByObstacle && obstacleDetectedCurrent) {
        currentLine2 = "Yolu Temizleyin"; // Engel hala orada
      } else if (currentState == INVALID_CARD_DETECTED && obstacleDetectedCurrent) {
        currentLine2 = "Giris Engellendi";
      } else {
        currentLine2 = "Yolu Temizleyin"; // Genel engel mesajı
      }
    }
    // Devam Etme Bildirimi (Öncelik 3 - engel yoksa ve kısa mesaj yoksa)
    else if (currentState == STOPPED && wasStoppedByObstacle && !obstacleDetectedCurrent) {
      currentLine1 = "Engel Temizlendi";
      currentLine2 = "Devam icin Buton";
    }
    // Normal Çalışma Mesajları (Öncelik 4)
    else {
      switch (currentState) {
        case CARD_READ_PROCESSING:
          if (isForwardSequenceInitiatedByRFID && (now - actionStartTimestamp < YETKILI_KART_DISPLAY_TIME)) {
            currentLine1 = "Yetkili Kart";
            currentLine2 = "Islem Basliyor...";
          } else {
            currentLine1 = "Rampa Aciliyor"; // Genel işlem başlangıcı
            currentLine2 = "Lutfen Bekleyin";
          }
          break;
        case STEPPER_MOVING_FORWARD:
        case OPENING_RAMP:
          currentLine1 = "Rampa Aciliyor";
          currentLine2 = wasStoppedByObstacle ? "Devam Ediyor..." : "Lutfen Bekleyin.";
          break;
        case CLOSING_RAMP:
        case STEPPER_MOVING_BACKWARD:
          currentLine1 = "Rampa Kapaniyor";
          currentLine2 = wasStoppedByObstacle ? "Devam Ediyor..." : "Lutfen Bekleyin.";
          break;
        case INVALID_CARD_DETECTED:
          currentLine1 = "Gecersiz Kart";
          currentLine2 = "Rampa Baslatilamiyor";
          break;
        case STOPPED:
          if (ramp_manualMotorRunning) { // Manuel rampa kontrolü aktifse
            currentLine1 = "Manuel Rampa";
            currentLine2 = ramp_manualDirectionForward ? "Aciliyor..." : "Kapaniyor...";
          } else if (rampOpen) { // Rampa açıksa
            currentLine1 = "Rampa Acik";
            currentLine2 = "Kullanilabilir";
          } else { // Rampa kapalıysa (varsayılan durum)
            currentLine1 = "Rampa Kapali";
            currentLine2 = "Giris Bekliyor";
          }
          break;
        default: // Beklenmeyen bir durum
          currentLine1 = "Sistem Hatasi";
          currentLine2 = "Yeniden Baslatin";
          break;
      }
    }
  }

  // Engel ortaya çıkarsa veya durum STOPPED dışına değişirse kısa mesaj bayraklarını temizle
  if (obstacleDetectedCurrent || (currentState != STOPPED && (showBriefMessageRampaAcildi || showBriefMessageRampaKapandi))) {
    showBriefMessageRampaAcildi = false;
    showBriefMessageRampaKapandi = false;
  }

  // Gerçek LCD yazdırma mantığı (sadece değişiklik varsa yazdırır)
  if (currentLine1 != prevLine1 || currentLine2 != prevLine2) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(currentLine1);
    lcd.setCursor(0, 1); lcd.print(currentLine2);
    prevLine1 = currentLine1;
    prevLine2 = currentLine2;
    Serial.print(F("LCD Guncelleme: S1='")); Serial.print(currentLine1);
    Serial.print(F("', S2='")); Serial.print(currentLine2); Serial.println(F("'"));
  }
}


void stopAllMotorsAndGoToStopped(const char* reason) { // Tüm motorları durdurur ve sistemi STOPPED durumuna geçirir
  Serial.print(F("SISTEM DURDURMA: Sebep: ")); Serial.println(reason);
  motorRunning = false; // Step motoru durdur
  digitalWrite(PUL_PIN, LOW); // Step motor pals pinini LOW yap
  analogWrite(RPWM, 0); // Rampa motoru ileri PWM'ini kapat
  analogWrite(LPWM, 0); // Rampa motoru geri PWM'ini kapat

  if (ramp_manualMotorRunning) {
    ramp_manualMotorRunning = false;
    Serial.println(F("MANUEL_RAMPA_KONTROL: stopAllMotorsAndGoToStopped cagrisi nedeniyle durduruldu."));
  }
  currentState = STOPPED;

  showBriefMessageRampaAcildi = false;
  showBriefMessageRampaKapandi = false;
  Serial.println(F("SISTEM_DURUMU: -> DURDU (Tum motorlar kapali)"));
}

void setup() { // Arduino başlangıç fonksiyonu
  Serial.begin(9600);
  while (!Serial);
  Serial.println(F("========================================="));
  Serial.println(F("Gelistirilmis Rampa + Step V11 Basliyor..."));
  Serial.println(F("========================================="));
  Serial.print(F("IR Esik Degeri: ")); Serial.println(irThreshold);
  Serial.println(F("IR sensorunun kalibre edildiginden emin olun: Dusuk okuma = ENGEL YOK, Yuksek okuma = ENGEL VAR"));

  SPI.begin();
  Serial.println(F("- SPI Baslatildi."));

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(irAnalogPin, INPUT);
  pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
  pinMode(REN,  OUTPUT); pinMode(LEN,  OUTPUT);
  digitalWrite(REN, HIGH); digitalWrite(LEN, HIGH); // Rampa motor sürücüsünü etkinleştir
  Serial.println(F("- DC Motor (Rampa) & IR Sensor Pinleri Baslatildi."));

  // Lineer Aktüatör (Rampa) Manuel Kontrol Pinleri
  pinMode(rampDirButtonPin, INPUT_PULLUP);
  pinMode(rampStartStopButtonPin, INPUT_PULLUP);
  Serial.println(F("- Lineer Aktuator (Rampa) Manuel Kontrol Pinleri Baslatildi (Pinler 11, 12)."));

  pinMode(DIR_PIN, OUTPUT); pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(START_STOP_PIN, INPUT_PULLUP);
  motorDirection = true; // Başlangıçta step motor yönü ileri
  digitalWrite(DIR_PIN, motorDirection ? HIGH : LOW);
  digitalWrite(PUL_PIN, LOW);
  Serial.println(F("- Step Motor Pinleri Baslatildi (Varsayilan Yon Ileri)."));

  Timer1.initialize(200); // 200 mikrosaniye aralıkla (step motor pals frekansı için)
  Timer1.attachInterrupt(stepperTimerISR);
  Serial.println(F("- Timer1 (Step Pals) Baslatildi: 200us aralik."));

  Wire.begin(); lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("Sistem Basliyor"));
  Serial.println(F("- LCD Baslatildi."));
  delay(1000);

  rfid.PCD_Init(); delay(50);
  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print(F("- RFID MFRC522 Versiyon: 0x")); Serial.println(version, HEX);
  if (version != 0x00 && version != 0xFF) {
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max); // Anten kazancını maksimuma ayarla
    Serial.println(F("- RFID Anten Kazanci Maksimuma Ayarlandi."));
  } else {
    Serial.println(F("UYARI: MFRC522 bulunamadi!"));
    lcd.clear(); lcd.print(F("RFID HATA!")); lcd.setCursor(0,1); lcd.print(F("Kontrol Edin"));
  }

  updateLCD(); // LCD'yi ilk durum mesajlarıyla güncelle
  Serial.println(F("========================================="));
  Serial.println(F("Sistem Hazir. Giris bekleniyor..."));
  Serial.println(F("========================================="));
}

void handleStepperButtonsManual(unsigned long currentTime) { // Step motor manuel kontrol butonlarını yöneten fonksiyon
  static unsigned long lastStepperBtnCheckTime = 0;
  static bool currentDirButtonStateLocal = HIGH;
  static bool currentStartStopStateLocal = HIGH;

  if (currentTime - lastStepperBtnCheckTime < 20) return; // 20ms aralıkla kontrol
  lastStepperBtnCheckTime = currentTime;

  // Step Motor Başlat/Durdur Butonu (Pin 9)
  int startStopReading = digitalRead(START_STOP_PIN);
  if (startStopReading != lastStartStopState) {
    lastStartStopDebounceTime = currentTime;
  }
  if ((currentTime - lastStartStopDebounceTime) > debounceDelay) {
    if (startStopReading != currentStartStopStateLocal) {
      currentStartStopStateLocal = startStopReading;
      if (currentStartStopStateLocal == LOW) {
        motorRunning = !motorRunning;
        Serial.print(F("MANUEL_STEP_BTN(9): NEMA Baslat/Durdur. NEMA Motor Calismasi: "));
        Serial.println(motorRunning ? F("ACIK") : F("KAPALI"));
        if (!motorRunning) {
          digitalWrite(PUL_PIN, LOW);
        }
      }
    }
  }
  lastStartStopState = startStopReading;

  // Step Motor Yön Butonu (Pin 3)
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
        Serial.print(F("MANUEL_STEP_BTN(3): NEMA Yon Degistirildi. NEMA Yonu: "));
        Serial.println(motorDirection ? F("ILERI (DIR_PIN=HIGH)") : F("GERI (DIR_PIN=LOW)"));
      }
    }
  }
  lastDirButtonState = dirReading;
}

void handleRampButtonsManual(unsigned long currentTime) { // Rampa manuel kontrol butonlarını yöneten fonksiyon
  static unsigned long lastRampBtnCheckTime_local = 0;
  static bool currentRampDirButtonState_local = HIGH;
  static bool currentRampStartStopState_local = HIGH;

  if (currentTime - lastRampBtnCheckTime_local < 20) return; // 20ms aralıkla kontrol
  lastRampBtnCheckTime_local = currentTime;

  // Manuel rampa kontrolü sadece sistem 'STOPPED' durumundaysa ve engel nedeniyle durmamışsa çalışır.
  if (currentState != STOPPED || wasStoppedByObstacle) {
    if (ramp_manualMotorRunning) {
      analogWrite(RPWM, 0);
      analogWrite(LPWM, 0);
      ramp_manualMotorRunning = false;
      Serial.println(F("MANUEL_RAMPA_KONTROL: Sistem STOPPED durumundan ciktigi veya engel oldugu icin otomatik durduruldu."));
    }
    return;
  }

  // Rampa Başlat/Durdur Butonu Mantığı (Pin 12)
  int startStopReading = digitalRead(rampStartStopButtonPin);
  if (startStopReading != lastRampStartStopButtonState_manual) {
    lastRampStartStopDebounceTime_manual = currentTime;
  }
  if ((currentTime - lastRampStartStopDebounceTime_manual) > debounceDelay) {
    if (startStopReading != currentRampStartStopState_local) {
      currentRampStartStopState_local = startStopReading;
      if (currentRampStartStopState_local == LOW) {
        ramp_manualMotorRunning = !ramp_manualMotorRunning;
        Serial.print(F("MANUEL_RAMPA_BTN(12): Baslat/Durdur. Rampa Manuel Motor Calismasi: "));
        Serial.println(ramp_manualMotorRunning ? F("ACIK") : F("KAPALI"));

        if (ramp_manualMotorRunning) {
          if (ramp_manualDirectionForward) {
            analogWrite(RPWM, motorSpeedRamp);
            analogWrite(LPWM, 0);
            Serial.println(F("MANUEL_RAMPA_KONTROL: ILERI hareket (Aciliyor)"));
          } else {
            analogWrite(RPWM, 0);
            analogWrite(LPWM, motorSpeedRamp);
            Serial.println(F("MANUEL_RAMPA_KONTROL: GERI hareket (Kapaniyor)"));
          }
        } else {
          analogWrite(RPWM, 0);
          analogWrite(LPWM, 0);
          Serial.println(F("MANUEL_RAMPA_KONTROL: Durduruldu."));
        }
      }
    }
  }
  lastRampStartStopButtonState_manual = startStopReading;

  // Rampa Yön Butonu Mantığı (Pin 11)
  int dirReading = digitalRead(rampDirButtonPin);
  if (dirReading != lastRampDirButtonState_manual) {
    lastRampDirDebounceTime_manual = currentTime;
  }
  if ((currentTime - lastRampDirDebounceTime_manual) > debounceDelay) {
    if (dirReading != currentRampDirButtonState_local) {
      currentRampDirButtonState_local = dirReading;
      if (currentRampDirButtonState_local == LOW) {
        if (!ramp_manualMotorRunning) { // Yönü sadece motor manuel olarak çalışmıyorken değiştir
          ramp_manualDirectionForward = !ramp_manualDirectionForward;
          Serial.print(F("MANUEL_RAMPA_BTN(11): Yon Degistirildi. Rampa Manuel Yonu: "));
          Serial.println(ramp_manualDirectionForward ? F("ILERI (Acma)") : F("GERI (Kapama)"));
        } else {
          Serial.println(F("MANUEL_RAMPA_BTN(11): Rampa motoru manuel calisirken yon degistirilemez. Once motoru durdurun."));
        }
      }
    }
  }
  lastRampDirButtonState_manual = dirReading;
}


void loop() { // Ana döngü fonksiyonu
  unsigned long now = millis();

  handleStepperButtonsManual(now);
  handleRampButtonsManual(now);

  static unsigned long lastIRCheckTime = 0;
  if (now - lastIRCheckTime > 100) { // 100ms'de bir IR sensörünü kontrol et
    int irReading = analogRead(irAnalogPin);
    bool prevObstacleState = obstacleDetectedCurrent;
    obstacleDetectedCurrent = (irReading > irThreshold);

    if (obstacleDetectedCurrent && !prevObstacleState) { // Yeni bir engel algılandıysa
      lastObstacleTimestamp = now;
      Serial.print(F("IR MANTIGI: Engel ALGILANDI. Analog: ")); Serial.print(irReading);
      Serial.print(F(" (Esik: ")); Serial.print(irThreshold); Serial.println(F(")"));

      bool isActiveMotorOpState = (currentState == OPENING_RAMP ||
                                   currentState == CLOSING_RAMP ||
                                   currentState == STEPPER_MOVING_FORWARD ||
                                   currentState == STEPPER_MOVING_BACKWARD);
      if (isActiveMotorOpState && !wasStoppedByObstacle) { // Aktif motor işlemi varsa ve daha önce engel nedeniyle durdurulmadıysa
        Serial.print(F("ENGEL: Aktif durum sirasinda algilandi: ")); Serial.println(currentState);
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
        
        Serial.print(F("ENGEL: Kaydedilen kesintiye ugrayan durum: ")); Serial.println(interruptedState);
        Serial.print(F("ENGEL: Kalan sure: ")); Serial.println(remainingOperationTime);
        
        stopAllMotorsAndGoToStopped("Motor Calismasi Sirasinda Engel Algilandi");
      }
    } else if (!obstacleDetectedCurrent && prevObstacleState) { // Engel temizlendiyse
      Serial.print(F("IR MANTIGI: Engel TEMIZLENDI. Analog: "));
      Serial.print(irReading);
      Serial.print(F(" (Esik: ")); Serial.print(irThreshold); Serial.println(F(")"));
    }
    lastIRCheckTime = now;
  }

  static unsigned long lastIRPeriodicLogTime = 0;
  if (now - lastIRPeriodicLogTime >= 1000) { // Saniyede bir IR durumunu logla
    int currentIRReadingForLog = analogRead(irAnalogPin);
    bool isObstacleForLog = (currentIRReadingForLog > irThreshold);
    Serial.print(F("IR DURUMU (1s): Deger = ")); Serial.print(currentIRReadingForLog);
    if (isObstacleForLog) Serial.println(F(" - Engel ALGILANDI"));
    else Serial.println(F(" - Yol temiz"));
    lastIRPeriodicLogTime = now;
  }
  
  static unsigned long lastRFIDResetTime = 0;
  if (now - lastRFIDResetTime > 60000) { // 60 saniyede bir RFID modülünü periyodik olarak resetle
    Serial.println(F("RFID: Periyodik PCD_Reset ve PCD_Init yapiliyor."));
    rfid.PCD_Reset(); delayMicroseconds(100); rfid.PCD_Init();
    rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
    lastRFIDResetTime = now; Serial.println(F("RFID: Periyodik reset tamamlandi."));
  }
  
  static unsigned long lastRFIDCheckTime = 0;
  // RFID okuması uygun koşullarda yapılır
  if (currentState == STOPPED && !wasStoppedByObstacle && !ramp_manualMotorRunning) {
    if (!rampOpen && (now - lastRFIDCheckTime > 200)) { // Rampa kapalıysa ve son okumadan 200ms geçtiyse
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        String currentCardUID = getUIDString(rfid.uid);
        Serial.print(F("RFID: Yeni kart algilandi. ")); printUID(rfid.uid);

        if (now - lastSuccessfulRFIDReadTime < rfidReadCooldown && currentCardUID == lastReadCardUID) {
          Serial.println(F("RFID: Ayni kart bekleme suresi icinde okundu. Yok sayiliyor."));
        } else {
          lastSuccessfulRFIDReadTime = now;
          lastReadCardUID = currentCardUID;

          if (isCardAuthorized(currentCardUID)) {
            if (!obstacleDetectedCurrent) {
              Serial.println(F("RFID: Yetkili kart. Islem baslatiliyor."));
              isForwardSequenceInitiatedByRFID = true;
              currentState = CARD_READ_PROCESSING;
              actionStartTimestamp = now;
              wasStoppedByObstacle = false;
              remainingOperationTime = 0;
              Serial.println(F("SISTEM_DURUMU: -> KART_OKUMA_ISLEMI (RFID ile)"));
            } else {
              Serial.println(F("RFID: Yetkili kart, ancak ENGEL var. Islem engellendi."));
              currentState = INVALID_CARD_DETECTED;
              actionStartTimestamp = now;
            }
          } else {
            Serial.println(F("RFID: Yetkisiz kart."));
            currentState = INVALID_CARD_DETECTED;
            actionStartTimestamp = now;
          }
        }
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
      }
      lastRFIDCheckTime = now;
    }
  }

  static unsigned long lastMainButtonCheckTime = 0;
  static bool prevMainButtonState = HIGH;
  if (now - lastMainButtonCheckTime > 50) { // 50ms'de bir ana butonu kontrol et
    bool currentMainButtonState = digitalRead(buttonPin);
    if (currentMainButtonState == LOW && prevMainButtonState == HIGH) { // Butona yeni basıldıysa
      Serial.println(F("ANA_BTN(10): Basildi."));
      showBriefMessageRampaAcildi = false;
      showBriefMessageRampaKapandi = false;

      if (ramp_manualMotorRunning) {
        ramp_manualMotorRunning = false;
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
        Serial.println(F("MANUEL_RAMPA_KONTROL: Ana butona basildi, manuel rampa kontrolu durduruldu."));
      } else if (currentState == STOPPED) {
        if (wasStoppedByObstacle) {
          if (!obstacleDetectedCurrent) {
            Serial.println(F("ANA_BTN(10): Engel temizlendikten sonra isleme devam ediliyor."));
            currentState = interruptedState;
            actionStartTimestamp = now;
            
            if (currentState == STEPPER_MOVING_FORWARD) {
              motorDirection = true;
              digitalWrite(DIR_PIN, HIGH);
              motorRunning = true;
              Serial.println(F("DEVAM_AYARI: Step ILERI, DIR=HIGH, motorRunning=true"));
            } else if (currentState == STEPPER_MOVING_BACKWARD) {
              motorDirection = false;
              digitalWrite(DIR_PIN, LOW);
              motorRunning = true;
              Serial.println(F("DEVAM_AYARI: Step GERI, DIR=LOW, motorRunning=true"));
            }
            Serial.print(F("SISTEM_DURUMU: -> DEVAM EDILIYOR: ")); Serial.println(currentState);
            Serial.print(F("Kalan sure: ")); Serial.println(remainingOperationTime);
          } else {
            Serial.println(F("ANA_BTN(10): Engel hala mevcut. Devam edilemiyor."));
          }
        } else if (!obstacleDetectedCurrent) {
          if (!rampOpen) { // Rampa kapalıysa, ileri yönlü işlem başlat
            Serial.println(F("ANA_BTN(10): ILERI yonlu islem baslatiliyor."));
            isForwardSequenceInitiatedByRFID = false;
            currentState = CARD_READ_PROCESSING;
            actionStartTimestamp = now;
            wasStoppedByObstacle = false;
            remainingOperationTime = 0; interruptedState = STOPPED;
          } else { // Rampa açıksa, geri yönlü işlem başlat
            Serial.println(F("ANA_BTN(10): GERI yonlu islem baslatiliyor."));
            currentState = CLOSING_RAMP;
            actionStartTimestamp = now;
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }
        } else {
          Serial.println(F("ANA_BTN(10): Basildi, ancak ENGEL var. Yeni islem engellendi."));
          currentState = INVALID_CARD_DETECTED;
          actionStartTimestamp = now;
        }
      } else { // Sistem aktif bir işlemdeyse, butona basılırsa her şeyi durdur
        stopAllMotorsAndGoToStopped("Ana Buton (10) aktif/hata durumunda basildi");
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


  switch (currentState) { // Mevcut sistem durumuna göre işlem yap
    case CARD_READ_PROCESSING:
      if (now - actionStartTimestamp > cardProcessingDisplayTime) {
        if (!obstacleDetectedCurrent) {
          currentState = STEPPER_MOVING_FORWARD;
          actionStartTimestamp = now;
          motorDirection = true;
          digitalWrite(DIR_PIN, HIGH);
          motorRunning = true;
          Serial.println(F("SISTEM_DURUMU: KART_OKUMA_ISLEMI -> STEP_ILERI_HAREKET"));
          Serial.println(F("STEP_KONTROL: NEMA motor ILERI yonlu islem baslatildi."));
        } else {
          Serial.println(F("SISTEM_DURUMU: KART_OKUMA_ISLEMI -> KONTROL SONRASI ENGEL ALGILANDI. Islem engelleniyor."));
          currentState = INVALID_CARD_DETECTED;
          actionStartTimestamp = now;
        }
      }
      analogWrite(RPWM, 0); // Bu durumda rampa motorları durmalı
      analogWrite(LPWM, 0);
      break;

    case INVALID_CARD_DETECTED:
      if (now - actionStartTimestamp > invalidCardDisplayTime) {
        currentState = STOPPED;
        Serial.println(F("SISTEM_DURUMU: GECERSIZ_KART -> DURDU (Gosterim suresi doldu)"));
      }
      analogWrite(RPWM, 0); analogWrite(LPWM, 0); // Motorlar durmalı
      break;

    case STEPPER_MOVING_FORWARD:
      { 
        unsigned long durationToUseStepperFwd = wasStoppedByObstacle ? remainingOperationTime : runDurationStepper;
        if (!motorRunning && wasStoppedByObstacle) motorRunning = true;
        if ((now - actionStartTimestamp) >= durationToUseStepperFwd) {
          Serial.print(F("STEP_KONTROL: NEMA ILERI sure doldu. Kullanilan sure: "));
          Serial.println(durationToUseStepperFwd);
          motorRunning = false;
          digitalWrite(PUL_PIN, LOW);
          
          if (wasStoppedByObstacle) {
            Serial.println(F("STEP_KONTROL: Devam ettirilen NEMA ILERI hareketi tamamlandi."));
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }

          if (!obstacleDetectedCurrent) {
            currentState = OPENING_RAMP;
            actionStartTimestamp = now;
            Serial.print(F("SISTEM_DURUMU: STEP_ILERI_HAREKET -> RAMPA_ACILIYOR (Baslatan: "));
            Serial.println(isForwardSequenceInitiatedByRFID ? F("RFID)") : F("Buton)"));
          } else {
            Serial.println(F("SISTEM_DURUMU: STEP_ILERI_HAREKET -> ENGEL. Durduruluyor."));
            stopAllMotorsAndGoToStopped("NEMA ileri sonrasi, rampa acilmadan engel");
          }
        }
      }
      analogWrite(RPWM, 0); // Bu durumda rampa motorları durmalı
      analogWrite(LPWM, 0);
      break;

    case OPENING_RAMP:
      { 
        unsigned long durationToUseRampOpen = wasStoppedByObstacle ? remainingOperationTime : runDurationRamp;
        if ((now - actionStartTimestamp) < durationToUseRampOpen) {
          analogWrite(RPWM, motorSpeedRamp);
          analogWrite(LPWM, 0);
        } else {
          Serial.print(F("RAMPA_KONTROL: Rampa ACILDI. Kullanilan sure: "));
          Serial.println(durationToUseRampOpen);
          analogWrite(RPWM, 0); analogWrite(LPWM, 0);
          rampOpen = true;
          rampOpenedByRFIDGlobal = isForwardSequenceInitiatedByRFID;
          if (wasStoppedByObstacle) {
            Serial.println(F("RAMPA_KONTROL: Devam ettirilen RAMPA ACMA islemi tamamlandi."));
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }
          
          showBriefMessageRampaAcildi = true;
          showBriefMessageRampaKapandi = false;
          briefMessageDisplayUntil = now + BRIEF_MESSAGE_DURATION;
          
          currentState = STOPPED;
          Serial.print(F("SISTEM_DURUMU: RAMPA_ACILIYOR -> DURDU. Rampa RFID ile acildi: "));
          Serial.println(rampOpenedByRFIDGlobal ? F("evet") : F("hayir"));
        }
      }
      break;

    case CLOSING_RAMP:
      { 
        unsigned long durationToUseRampClose = wasStoppedByObstacle ? remainingOperationTime : runDurationRamp;
        if ((now - actionStartTimestamp) < durationToUseRampClose) {
          analogWrite(RPWM, 0);
          analogWrite(LPWM, motorSpeedRamp);
        } else {
          Serial.print(F("RAMPA_KONTROL: Rampa KAPANDI. Kullanilan sure: "));
          Serial.println(durationToUseRampClose);
          analogWrite(RPWM, 0); analogWrite(LPWM, 0);
          rampOpen = false;
          
          if (wasStoppedByObstacle) {
            Serial.println(F("RAMPA_KONTROL: Devam ettirilen RAMPA KAPAMA islemi tamamlandi."));
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }

          if (!obstacleDetectedCurrent) {
            currentState = STEPPER_MOVING_BACKWARD;
            actionStartTimestamp = now;
            motorDirection = false;
            digitalWrite(DIR_PIN, LOW);
            motorRunning = true;
            Serial.println(F("SISTEM_DURUMU: RAMPA_KAPANIYOR -> STEP_GERI_HAREKET"));
            Serial.println(F("STEP_KONTROL: NEMA motor GERI yonlu islem baslatildi."));
          } else {
            Serial.println(F("SISTEM_DURUMU: RAMPA_KAPANIYOR -> ENGEL. Durduruluyor."));
            stopAllMotorsAndGoToStopped("Rampa kapandiktan sonra, NEMA geri oncesi engel");
          }
        }
      }
      break;

    case STEPPER_MOVING_BACKWARD:
      { 
        unsigned long durationToUseStepperBwd = wasStoppedByObstacle ? remainingOperationTime : runDurationStepper;
        if (!motorRunning && wasStoppedByObstacle) motorRunning = true;
        if ((now - actionStartTimestamp) >= durationToUseStepperBwd) {
          Serial.print(F("STEP_KONTROL: NEMA GERI sure doldu. Kullanilan sure: "));
          Serial.println(durationToUseStepperBwd);
          motorRunning = false;
          digitalWrite(PUL_PIN, LOW);

          if (wasStoppedByObstacle) {
            Serial.println(F("STEP_KONTROL: Devam ettirilen NEMA GERI hareketi tamamlandi."));
            wasStoppedByObstacle = false; remainingOperationTime = 0; interruptedState = STOPPED;
          }
          
          showBriefMessageRampaKapandi = true;
          showBriefMessageRampaAcildi = false;
          briefMessageDisplayUntil = now + BRIEF_MESSAGE_DURATION;

          currentState = STOPPED;
          isForwardSequenceInitiatedByRFID = false;
          rampOpenedByRFIDGlobal = false;
          Serial.println(F("SISTEM_DURUMU: STEP_GERI_HAREKET -> DURDU (Geri Yonlu Islem Tamamlandi)"));
        }
      }
      analogWrite(RPWM, 0); analogWrite(LPWM, 0); // Bu durumda rampa motorları durmalı
      break;

    case STOPPED:
      if (!ramp_manualMotorRunning) { // Manuel rampa çalışmıyorsa
        analogWrite(RPWM, 0); // Rampa motorlarını kapalı tut
        analogWrite(LPWM, 0);
      }
      break;

    default:
      // Beklenmedik durum
      break;
  }

  static unsigned long lastLCDUpdateTime = 0;
  if (now - lastLCDUpdateTime > 250) { // 250ms'de bir LCD'yi güncelle
    updateLCD();
    lastLCDUpdateTime = now;
  }
  
  static unsigned long lastStatusReportTime = 0;
  if (now - lastStatusReportTime > 60000) { // 60 saniyede bir durum raporu yazdır
    Serial.println(F("--- Durum Raporu ---"));
    Serial.print(F("  Zaman Damgasi: ")); Serial.println(now);
    Serial.print(F("  Sistem Durumu: "));
    switch(currentState) {
      case STOPPED: Serial.println(F("DURDU")); break;
      case CARD_READ_PROCESSING: Serial.println(F("KART_OKUMA_ISLEMI")); break;
      case INVALID_CARD_DETECTED: Serial.println(F("GECERSIZ_KART")); break;
      case STEPPER_MOVING_FORWARD: Serial.println(F("STEP_ILERI_HAREKET")); break;
      case OPENING_RAMP: Serial.println(F("RAMPA_ACILIYOR")); break;
      case CLOSING_RAMP: Serial.println(F("RAMPA_KAPANIYOR")); break;
      case STEPPER_MOVING_BACKWARD: Serial.println(F("STEP_GERI_HAREKET")); break;
      default: Serial.println(F("BILINMEYEN")); break;
    }
    Serial.print(F("  Rampa Fiziksel Olarak Acik: ")); Serial.println(rampOpen ? F("Evet") : F("Hayir"));
    Serial.print(F("  Engel Algilandi: ")); Serial.println(obstacleDetectedCurrent ? F("Evet") : F("Hayir"));
    if (obstacleDetectedCurrent && lastObstacleTimestamp > 0) {
      Serial.print(F("  Engel Ilk Fark Edildiginden Beri Gecen Sure: "));
      Serial.print(now - lastObstacleTimestamp); Serial.println(F(" ms"));
    }
    Serial.print(F("  Engel Tarafindan Durduruldu mu: "));
    Serial.println(wasStoppedByObstacle ? F("Evet") : F("Hayir"));
    if(wasStoppedByObstacle){
      Serial.print(F("  Kalan Islem Suresi: ")); Serial.println(remainingOperationTime);
    }
    Serial.print(F("  NEMA Motor Calisiyor (Otomatik): ")); Serial.println(motorRunning ? F("Evet") : F("Hayir"));
    Serial.print(F("  NEMA Motor Yonu (true=ileri): ")); Serial.println(motorDirection);
    Serial.print(F("  Manuel Rampa Motoru Calisiyor: ")); Serial.println(ramp_manualMotorRunning ? F("Evet") : F("Hayir"));
    Serial.print(F("  Manuel Rampa Yonu (true=ileri/acma): ")); Serial.println(ramp_manualDirectionForward);
    Serial.print(F("  KisaMsg Acildi: ")); Serial.print(showBriefMessageRampaAcildi);
    Serial.print(F(" Kapandi: ")); Serial.print(showBriefMessageRampaKapandi);
    Serial.print(F(" Sure Sonu: ")); Serial.println(briefMessageDisplayUntil);
    Serial.println(F("---------------------------"));
    lastStatusReportTime = now;
  }
}