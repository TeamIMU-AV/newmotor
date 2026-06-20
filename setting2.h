#ifndef SETTINGS_H
#define SETTINGS_H

// -------------------------------------------------------
// PIN TANIMLARI
// -------------------------------------------------------
#define BRAKE_PIN 3
#define MOTOR_PIN 9
#define GEAR_PIN  8
#define READ_PIN  14  // A0

// -------------------------------------------------------
// HIZ AYARLARI
// -------------------------------------------------------
#define SPEED_RES     100   // PWM çözünürlüğü (0-100)
#define SPEED_OUT_MIN 180   // Motor minimum PWM (harekete başlama eşiği)
#define SPEED_OUT_MAX 255   // Motor maksimum PWM
#define SPEED_IN_MIN  10    // Encoder/sensör minimum okuması
#define SPEED_IN_MAX  100   // Encoder/sensör maksimum okuması

// -------------------------------------------------------
// FREN SERVO AYARLARI
// -------------------------------------------------------
#define BRAKE_ON  750   // Fren basılı servo sinyali (us)
#define BRAKE_OFF 2250  // Fren serbest servo sinyali (us)

// -------------------------------------------------------
// ENUM TANIMLARI
// -------------------------------------------------------
enum CarState {
    STATE_IDLE,         // Araç duruyor, fren %100, motor 0
    STATE_PREPARING,    // Fren kademeli bırakılıyor, vites seçiliyor
    STATE_CRUISING,     // Serbest sürüş, PI kontrolcü aktif
    STATE_DECELERATING  // Yavaşlama (COASTING veya BRAKING)
};

enum SubState {
    SUB_NONE,
    SUB_COASTING,
    SUB_BRAKING
};

enum GearState {
    BACKWARD,
    FORWARD
};

#endif // SETTINGS_H
