#include <Arduino.h>
#include "PacketSerial.h"
#include "ServoTimer2.h"
#include "Settings2.h"

#define DEBUG

#define MY_NODE_ID 2

PacketSerial mySerial;
ServoTimer2 brakeServo;

// -------------------------------------------------------
// DEĞİŞKENLER & DURUM KONTROLÜ
// -------------------------------------------------------
CarState carState = STATE_IDLE;
SubState subState = SUB_NONE;

int16_t target_speed  = 0;   // Mega'dan gelen hedef hız (PacketSerial üzerinden)
int8_t  targetSpeed   = 0;   // State machine'in kullandığı tip (orijinal koddaki gibi)
int8_t  currentSpeed  = 0;
GearState lastGear    = FORWARD;

// PI kontrolcü
float integral_hata = 0.0;
float Kp = 2.5;
float Ki = 0.5;
float dt = 0.1;
static int onceki_pwm = 0;  // PWM rampa sınırlayıcı

// KALKIŞ İVMESİ (Sahada bu rakamı değiştirerek kalkış sertliğini ayarlayabilirsin)
int RAMPA_ADIMI = 15;

// PREPARING zamanlayıcısı (Debriyaj Etkisi)
unsigned long prepareStartTime = 0;
const unsigned long PREPARE_DURATION_MS = 400;  // 400ms içinde fren sıfıra iner
int prepareBrakePercent = 100;                   // fren yüzdesi kademeli düşer

// Ana döngü zamanlayıcısı
unsigned long eski_zaman = 0;

// Sensör gürültüsüne / ani sıçramaya karşı güvenlik payı: DECELERATING'den
// IDLE'a geçmeden önce hızın art arda kaç döngü boyunca sıfır göründüğünü
// sayıyoruz. Tek bir hatalı/gürültülü okuma yüzünden erken IDLE'a (ve
// dolayısıyla başka bir state'e) geçilmesini önler.
uint8_t zeroSpeedStreak = 0;
const uint8_t ZERO_SPEED_CONFIRM_COUNT = 3; // ~3 x 100ms = 300ms boyunca teyit

// -------------------------------------------------------
// PACKETSERIAL HABERLEŞME (main (6).cpp formatı)
// -------------------------------------------------------
// Paket yapısı Node 2 için: [ID(1)][target_speed(2)][CRC32(4)] = 7 byte
unsigned long lastSendTime = 0;
unsigned long lastPacketReceivedTime = 0;
const unsigned long FAILSAFE_TIMEOUT_MS = 500;

void onPacketReceived(const uint8_t* buffer, size_t size);
void sendFeedbackData();
uint32_t calculateCRC32(const uint8_t *data, size_t length);

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (size_t j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

// -------------------------------------------------------
// DEBUG İŞLEMLERİ
// -------------------------------------------------------
#ifdef DEBUG
bool ledState = false;
void triggerLed() {
    digitalWrite(LED_BUILTIN, ledState);
    ledState = !ledState;
}
#endif

// -------------------------------------------------------
// DONANIM SOYUTLAMA KATMANI (FİZİKSEL ÇIKIŞLAR)
// -------------------------------------------------------
// Arduino'nun map() fonksiyonu long tabanlıdır ve ondalıklı değerlerde
// hassasiyet kaybına yol açar. Float hassasiyetiyle çalışan kendi
// versiyonumuzu kullanıyoruz.
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setGear(GearState _state) {
    digitalWrite(GEAR_PIN, _state == FORWARD ? HIGH : LOW);
    lastGear = _state;
}

void setBrakeDynamic(int yuzde) {
    int servo_sinyali = map(yuzde, 0, 100, BRAKE_OFF, BRAKE_ON);
    brakeServo.write(servo_sinyali);
}

uint8_t getSpeedScaled() {
   float scaled = analogRead(READ_PIN) / 5.0;
   float mapped = mapFloat(scaled, SPEED_IN_MIN, SPEED_IN_MAX, 0, SPEED_RES);
    if (scaled < 5) return 0;
    return constrain((int)round(mapped), 0, SPEED_RES);

}

void setSpeedScaled(uint8_t _speed) {
    uint8_t speed = constrain(_speed, 0, SPEED_RES);
    if (speed == 0) {
        analogWrite(MOTOR_PIN, 0);
    } else {
        analogWrite(MOTOR_PIN, map(speed, 1, SPEED_RES, SPEED_OUT_MIN, SPEED_OUT_MAX));
    }
}


// OTONOM KONTROL DÖNGÜSÜ (BEYİN) 

void handleControl() {
    float hata = abs(targetSpeed) - abs(currentSpeed);
    int pwm_komut        = 0;
    int fren_komut_yuzdesi = 0;

    // --- STATE (DURUM) GEÇİŞ KARARLARI ---
    switch (carState) {

        case STATE_IDLE:
            if (targetSpeed != 0) {
                carState           = STATE_PREPARING;
                prepareStartTime   = millis();
                prepareBrakePercent = 100;

                if (targetSpeed > 0) setGear(FORWARD);
                if (targetSpeed < 0) setGear(BACKWARD);
            }
            break;

        case STATE_PREPARING:
            if (targetSpeed == 0) {
                carState = STATE_IDLE;
                break;
            }

            {
                unsigned long gecen = millis() - prepareStartTime;
                prepareBrakePercent = map(gecen, 0, PREPARE_DURATION_MS, 100, 0);
                prepareBrakePercent = constrain(prepareBrakePercent, 0, 100);

                if (gecen >= PREPARE_DURATION_MS) {
                    prepareBrakePercent = 0;
                    carState            = STATE_CRUISING;
                    integral_hata       = 0.0;
                }
            }
            break;

        case STATE_CRUISING:
            if (targetSpeed == 0) {
                // DÜZELTME: Doğrudan IDLE'a atlamak ani %100 fren anlamına
                // geliyordu (örn. failsafe tetiklendiğinde). Artık önce
                // DECELERATING/BRAKING'e girip kademeli yavaşlama yapılıyor;
                // araç gerçekten durduğunda DECELERATING zaten kendisi
                // IDLE'a düşürüyor 
                carState = STATE_DECELERATING;
                subState = SUB_BRAKING;
                zeroSpeedStreak = 0;
            } else if (hata < -1.0) {
                carState = STATE_DECELERATING;
                subState = (abs(hata) > 8.0) ? SUB_BRAKING : SUB_COASTING;
                zeroSpeedStreak = 0;
            }
            break;

        case STATE_DECELERATING:
            if (abs(currentSpeed) == 0) {
                zeroSpeedStreak++;
            } else {
                zeroSpeedStreak = 0;
            }

            if (zeroSpeedStreak >= ZERO_SPEED_CONFIRM_COUNT) {
                
                carState = STATE_IDLE;
                subState = SUB_NONE;
                zeroSpeedStreak = 0;
            } else if (targetSpeed != 0 && hata >= -1.0) {
                
                carState = STATE_CRUISING;
                subState = SUB_NONE;
                zeroSpeedStreak = 0;
            } else {
                // Hâlâ hareket halindeyiz, kademeli yavaşlamaya devam et.
                subState = (abs(hata) > 8.0) ? SUB_BRAKING : SUB_COASTING;
            }
            break;
    }

    // --- DURUMLARIN FİZİKSEL ÇIKTILARI ---
    switch (carState) {

        case STATE_IDLE:
            fren_komut_yuzdesi = 100;
            setSpeedScaled(0);
            integral_hata = 0.0;
            onceki_pwm = 0;
            break;

        case STATE_PREPARING:
            fren_komut_yuzdesi = prepareBrakePercent;
            setSpeedScaled(0);
            onceki_pwm = 0;
            break;

        case STATE_CRUISING:
            fren_komut_yuzdesi = 0;

            integral_hata += hata * dt;
            float pi_cikis = (Kp * hata) + (Ki * integral_hata);
            pwm_komut = (int)constrain(pi_cikis * 2, 0, SPEED_RES);

            // YUMUŞAK KALKIŞ (RAMPA) KONTROLÜ
            if (pwm_komut > onceki_pwm + RAMPA_ADIMI) {
                pwm_komut = onceki_pwm + RAMPA_ADIMI;
            }

            onceki_pwm = pwm_komut;
            setSpeedScaled(pwm_komut);
            break;

        case STATE_DECELERATING:
            setSpeedScaled(0);
            integral_hata = 0.0;

            if (subState == SUB_BRAKING) {
                fren_komut_yuzdesi = (int)min(100.0f, abs(hata) * 6.0f);
            } else {
                fren_komut_yuzdesi = 0;
            }
            break;
    }

    setBrakeDynamic(fren_komut_yuzdesi);

// --- SERİ PORT BİLGİLENDİRME ---
#ifdef DEBUG
    const char* stateStr[] = {"IDLE", "PREPARING", "CRUISING", "DECELERATING"};
    const char* subStr[]   = {"NONE", "COASTING", "BRAKING"};
    Serial.print("DURUM: ");    Serial.print(stateStr[carState]);
    Serial.print(" | SUB: ");   Serial.print(subStr[subState]);
    Serial.print(" | Hedef: "); Serial.print(targetSpeed);
    Serial.print(" | Anlik: "); Serial.print(currentSpeed);
    Serial.print(" | Motor: "); Serial.print(pwm_komut);
    Serial.print(" | Fren: %"); Serial.println(fren_komut_yuzdesi);
#endif
}

// -------------------------------------------------------
// PACKETSERIAL ALICI / VERİCİ
// -------------------------------------------------------
void onPacketReceived(const uint8_t* buffer, size_t size) {
  // Beklenen boyut: ID(1) + target_speed(2) + CRC32(4) = 7 byte
  if (size < 5) return;

  uint32_t receivedCRC;
  memcpy(&receivedCRC, buffer + (size - 4), 4);
  uint32_t calculatedCRC = calculateCRC32(buffer, size - 4);

  if (receivedCRC == calculatedCRC) {
    uint8_t incoming_id = buffer[0];

    if (incoming_id == MY_NODE_ID) {
      lastPacketReceivedTime = millis(); // Failsafe zamanlayıcısını resetle

      memcpy(&target_speed, buffer + 1, 2);

#ifdef DEBUG
      triggerLed();
#endif

      // State machine int8_t bekliyor (orijinal koddaki gibi), bu yüzden
      // gelen int16_t değeri SPEED_RES aralığına sıkıştırıp aktarıyoruz.
      targetSpeed = (int8_t)constrain(target_speed, -SPEED_RES, SPEED_RES);
    }
  }
}

void sendFeedbackData() {
  uint8_t payload[3];
  payload[0] = MY_NODE_ID;
  int16_t feedback_value = currentSpeed;
  memcpy(payload + 1, &feedback_value, 2);

  uint32_t crc = calculateCRC32(payload, 3);

  uint8_t packet[7];
  memcpy(packet, payload, 3);
  memcpy(packet + 3, &crc, 4);

  mySerial.send(packet, sizeof(packet));
}

// -------------------------------------------------------
// KURULUM VE ANA DÖNGÜ
// -------------------------------------------------------
void setup() {
    Serial.begin(115200);
    mySerial.setStream(&Serial);
    mySerial.setPacketHandler(&onPacketReceived);

    brakeServo.attach(BRAKE_PIN);
    pinMode(MOTOR_PIN, OUTPUT);
    pinMode(GEAR_PIN,  OUTPUT);
    pinMode(READ_PIN,  INPUT);
    pinMode(11,        OUTPUT);

#ifdef DEBUG
    pinMode(LED_BUILTIN, OUTPUT);
#endif

    setSpeedScaled(0);
    setBrakeDynamic(100);
    setGear(FORWARD);

    // Failsafe zamanlayıcısını başlangıçta da set et; aksi halde
    // ilk 500ms içinde hiç paket gelmezse anında failsafe tetiklenir.
    lastPacketReceivedTime = millis();
}

void loop() {
    mySerial.update();

    currentSpeed = getSpeedScaled() * (lastGear == FORWARD ? 1 : -1);

    // HARDWARE FAILSAFE
    // Mega'dan FAILSAFE_TIMEOUT_MS süresinden uzun süre paket gelmezse
    // hedef hız sıfırlanır. State machine otomatik olarak STATE_IDLE'a
    // düşer ve freni devreye alır (orijinal mantık aynen korunuyor).
    if (millis() - lastPacketReceivedTime > FAILSAFE_TIMEOUT_MS) {
        targetSpeed = 0;
    }

    // Mega'ya hız geri bildirimini 50Hz'de gönder (main (6).cpp'deki gibi)
    if (millis() - lastSendTime > 20) {
        sendFeedbackData();
        lastSendTime = millis();
    }

    unsigned long su_anki_zaman = millis();
    if (su_anki_zaman - eski_zaman >= 100) {
        eski_zaman = su_anki_zaman;
        handleControl(); // 100ms'de bir beyni çalıştırır
    }
}
