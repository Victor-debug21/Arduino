#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 53   // RFID Modülü SPI Chip Select (SDA)
#define RST_PIN 49  // Reset pini

MFRC522 rfid(SS_PIN, RST_PIN);  // RFID modülü için nesne oluştur

void setup() {
  Serial.begin(9600);  // Seri haberleşmeyi başlat
  SPI.begin();         // SPI haberleşmesini başlat
  rfid.PCD_Init();     // RFID modülünü başlat
  
  Serial.println(" RFID Okuyucu Başlatıldı!");
  Serial.println(" Lütfen bir kart okutun...");
}

void loop() {
  // Eğer yeni bir RFID kartı algılanmazsa, döngüyü burada sonlandır
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  // Eğer karttan UID okunamıyorsa, döngüyü sonlandır
  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  // Kart UID’sini Seri Monitör'e yazdır
  Serial.print(" Kart UID: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? "0" : " ");  // Tek haneli sayılar için başına 0 ekle
    Serial.print(rfid.uid.uidByte[i], HEX);  // HEX formatında yazdır
  }
  Serial.println();

  // Kart okuma işlemi tamamlandı, kartı durdur
  rfid.PICC_HaltA();
}
