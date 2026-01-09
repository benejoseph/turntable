#ifndef FEEDBACK360_CONTROL_LIBRARY
#define FEEDBACK360_CONTROL_LIBRARY

#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>

#ifdef ARDUINO_AVR_UNO
#define MAX_INTERRUPT_NUM 2
#endif

#ifdef ARDUINO_AVR_LEONARDO
#define MAX_INTERRUPT_NUM 5
#endif

#ifdef ARDUINO_AVR_MEGA2560
#define MAX_INTERRUPT_NUM 6
#endif

#ifdef ARDUINO_SAMD_NANO_33_IOT
#define MAX_INTERRUPT_NUM 9
#endif

// If not listed above, leave it at 2.
// This is just a provisional value; some boards may require a larger value.
#ifndef MAX_INTERRUPT_NUM
#define MAX_INTERRUPT_NUM 2
#endif

// This means 31 - 12 = 19 bits to the left of the binary point
// and 12 bits to the right
// 1 signed bit obv
#define Q_FIXEDPOINT (8)
#define TOFIXEDPOINT(x) ((int32_t)(((double)(x)) * (1 << Q_FIXEDPOINT)))
#define F_FROMFIXEDPOINT(x) (((float)(x)) / (float)(1 << Q_FIXEDPOINT))
#define I_FROMFIXEDPOINT(x) ((x) / (1 << Q_FIXEDPOINT))
#define QONE (1 << Q_FIXEDPOINT)
#define QMUL(a, b)  ((int32_t)(((int64_t)((a) * (b))) >> Q_FIXEDPOINT))
#define QDIV(a, b)  (int32_t)(((((int64_t)(a)) << Q_FIXEDPOINT) / (b)))

class FeedBackServo
{
public:
    FeedBackServo(byte feedbackPinNumber);
    void setServoControl(byte servoPinNumber);
    void setKp(int32_t Kp, int32_t Ki, int32_t Kd);
    void setActive(bool isActive);
    void setTargetAngle(int32_t targetAngle);
    void update(unsigned long timestampMillis);
    void setSpeed(int32_t speed);
    void setThresholdDeg(int32_t thresholdDeg);
    int32_t getAngle() const;
    int32_t getErrorAngle() const;
    int32_t getTargetAngle() const;
    int32_t getTheta() const;
    int32_t getTurns() const;
    int32_t getErrorIntegral() const;
private:
    void checkPin(byte pinNumber);
    void handleFeedback();
    void updateAngleFromPWM();

    Servo parallax_;
    byte feedbackPinNumber_;
    byte interruptNumber_;

    int32_t Kp_ = TOFIXEDPOINT(1.0f);
    int32_t Ki_ = TOFIXEDPOINT(0.001);
    int32_t Kd_ = TOFIXEDPOINT(0.01);
    
    bool isActive_ = true;
    int32_t targetAngle_;

    volatile int32_t angle_ = 0;
    int32_t thetaPre_ = 0;
    uint16_t tHigh_ = 0, tLow_ = 0;
    unsigned long rise_ = 0, fall_ = 0;
    int32_t turns_ = 0;
    int32_t errorIntegral_ = 0;
    int32_t errorAngle_ = 0;
    int32_t thresholdDeg_ = TOFIXEDPOINT(4.0);
    unsigned long prevTimestampMillis_ = 0;

    volatile bool feedbackUpdated_ = false;

    static const int32_t UNITS_FC = TOFIXEDPOINT(360); // Full Circle
    static const int32_t DC_MIN;
    static const int32_t DC_MAX;
    static const int32_t DUTY_SCALE = QONE;
    static const int32_t Q2_MIN;
    static const int32_t Q3_MAX;

    static FeedBackServo* instances[MAX_INTERRUPT_NUM];
    static void isr0();
    static void isr1();
    static void isr2();
    static void isr3();
    static void isr4();
    static void isr5();
};

#endif
