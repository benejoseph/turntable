// FeedBack360 Servo Control Library 4 Arduino
// Controll.cpp
// Copyright © Hyoda Kazuaki
// Parallax Feedback 360° High Speed Servo is made by Parallax Inc.
// This library is released under the MIT License.

#include "FeedBackServo.h"

// Constants for duty cycle interpretation (based on Parallax spec)
const int32_t FeedBackServo::DC_MIN = TOFIXEDPOINT(0.029);
const int32_t FeedBackServo::DC_MAX = TOFIXEDPOINT(0.971);
const int32_t FeedBackServo::Q2_MIN = FeedBackServo::UNITS_FC / 4;
const int32_t FeedBackServo::Q3_MAX = FeedBackServo::Q2_MIN * 3;

#define STOP_PWM_MICROS (1490)

FeedBackServo* FeedBackServo::instances[MAX_INTERRUPT_NUM] = { nullptr };

FeedBackServo::FeedBackServo(byte feedbackPinNumber)
{
    // feedback pin number validation
    checkPin(feedbackPinNumber);
    feedbackPinNumber_ = feedbackPinNumber;

    // convert feedback pin number to interrupt number for use on attachInterrupt function
    interruptNumber_ = digitalPinToInterrupt(feedbackPinNumber_);

    if (interruptNumber_ < MAX_INTERRUPT_NUM) {
        instances[interruptNumber_] = this;
        switch (interruptNumber_)
        {
            case 0: attachInterrupt(0, isr0, CHANGE); break;
            case 1: attachInterrupt(1, isr1, CHANGE); break;
            case 2: attachInterrupt(2, isr2, CHANGE); break;
            case 3: attachInterrupt(3, isr3, CHANGE); break;
            case 4: attachInterrupt(4, isr4, CHANGE); break;
            case 5: attachInterrupt(5, isr5, CHANGE); break;
        }
    }
}

void FeedBackServo::setServoControl(byte servoPinNumber)
{
    // Servo control pin attach
    parallax_.attach(servoPinNumber);
}

void FeedBackServo::setKp(int32_t Kp, int32_t Ki, int32_t Kd)
{
    FeedBackServo::Kp_ = Kp;
    FeedBackServo::Ki_ = Ki;
    FeedBackServo::Kd_ = Kd;

}

void FeedBackServo::setActive(bool isActive)
{
    isActive_ = isActive;
}

void FeedBackServo::setTargetAngle(int32_t targetAngle)
{
    errorIntegral_ = 0;
    targetAngle_ = targetAngle;
}

void FeedBackServo::setThresholdDeg(int32_t thresholdDeg) 
{
    thresholdDeg_ = thresholdDeg; 
}

int32_t FeedBackServo::getAngle() const
{
    return angle_;
}

int32_t FeedBackServo::getErrorAngle() const
{
  return errorAngle_;
}

int32_t FeedBackServo::getTargetAngle() const 
{
  return targetAngle_;
}

int32_t FeedBackServo::getTheta() const 
{
  return thetaPre_;
}

int32_t FeedBackServo::getTurns() const 
{
    return turns_;
}

int32_t FeedBackServo::getErrorIntegral() const {

  return errorIntegral_;
}


void FeedBackServo::update(unsigned long timestampMillis)
{
    // Update angle based on the latest PWM feedback
    updateAngleFromPWM();
    int32_t deltaTimeMillis = timestampMillis - prevTimestampMillis_;
    prevTimestampMillis_ = timestampMillis;


    int32_t deltaTimeSeconds = QDIV((deltaTimeMillis << Q_FIXEDPOINT), TOFIXEDPOINT(1000.0));

    const int32_t prevErrorAngle = errorAngle_;
    errorAngle_ = targetAngle_ - angle_;

    if (isActive_ == false) return;

    if (abs(errorAngle_) <= thresholdDeg_)
    {
        parallax_.writeMicroseconds(STOP_PWM_MICROS);
        return;
    }

   const int32_t dError = errorAngle_ - prevErrorAngle;
   const int32_t dErrorDt = deltaTimeSeconds > 0 ? QDIV(dError, deltaTimeSeconds) : 0;

   if (abs(errorAngle_) < TOFIXEDPOINT(60.0f)) 
   {
       errorIntegral_ += QMUL(errorAngle_, deltaTimeSeconds);
   }
//    errorIntegral_ = constrain(-1000, 1000, errorIntegral_);

    // NOTE: Using simple P-control.
    // TODO: PID control may improve stability and response.
    //float output = errorAngle * Kp_ + errorIntegral_ * Ki_ + dError * Kd_;
    int32_t output = QMUL(errorAngle_, Kp_) + QMUL(errorIntegral_, Ki_) + QMUL(dErrorDt, Kd_);
    output = constrain(output, TOFIXEDPOINT(-200.0), TOFIXEDPOINT(200.0));
    int32_t theOffset = (output > 0) ? TOFIXEDPOINT(30.0) : TOFIXEDPOINT(-30.0);
    int16_t dutyCycle = STOP_PWM_MICROS - I_FROMFIXEDPOINT(output + theOffset);
    parallax_.writeMicroseconds(dutyCycle);
}

void FeedBackServo::setSpeed(int32_t theSpeed) {
  int32_t constrainedValue = constrain(theSpeed, TOFIXEDPOINT(-200), TOFIXEDPOINT(200));
  int16_t intConstrainedValue = I_FROMFIXEDPOINT(constrainedValue);
  parallax_.writeMicroseconds(STOP_PWM_MICROS - intConstrainedValue);
}

void FeedBackServo::checkPin(byte feedbackPinNumber)
{
// Check pin number
#ifdef ARDUINO_AVR_UNO
    if (feedbackPinNumber != 2 &&
        feedbackPinNumber != 3)
        exit(1);
#endif
#ifdef ARDUINO_AVR_LEONARDO
    if (feedbackPinNumber != 0 &&
        feedbackPinNumber != 1 &&
        feedbackPinNumber != 2 &&
        feedbackPinNumber != 3 &&
        feedbackPinNumber != 7)
        exit(1);
#endif
#ifdef ARDUINO_AVR_MEGA2560
    if (feedbackPinNumber != 2 &&
        feedbackPinNumber != 3 &&
        feedbackPinNumber != 18 &&
        feedbackPinNumber != 19 &&
        feedbackPinNumber != 20 &&
        feedbackPinNumber != 21)
        exit(1);
#endif
#ifdef ARDUINO_SAMD_NANO_33_IOT
    if (feedbackPinNumber != 2 &&
        feedbackPinNumber != 3 &&
        feedbackPinNumber != 9 &&
        feedbackPinNumber != 10 &&
        feedbackPinNumber != 11 &&
        feedbackPinNumber != 13 &&
        feedbackPinNumber != 15 && // A1
        feedbackPinNumber != 19 && // A5
        feedbackPinNumber != 21)   // A7
        exit(1);
#endif
}

void FeedBackServo::handleFeedback()
{
    // Interrupt Service Routine: triggered on pin CHANGE
    // Records timing only — actual decoding happens in main loop
    if (digitalRead(feedbackPinNumber_))
    {
        rise_ = micros();
        tLow_ = rise_ - fall_;
    }
    else
    {
        fall_ = micros();
        tHigh_ = fall_ - rise_;
        feedbackUpdated_ = true;
    }
}

void FeedBackServo::updateAngleFromPWM()
{
    if (!feedbackUpdated_) 
        return;
 
    feedbackUpdated_ = false;

    int32_t tCycle = tHigh_ + tLow_;
    if ((tCycle < 1000) || (tCycle > 1200))
        return;

    // (DUTY_SCALE * tHigh_) / (float)tCycle;
    int32_t num = DUTY_SCALE * tHigh_;
    int32_t denom = tCycle << Q_FIXEDPOINT;
    const int32_t dc = QDIV(num, denom);  

    // ((dc - DC_MIN) * UNITS_FC) / (DC_MAX - DC_MIN);
    num = QMUL(dc - DC_MIN, UNITS_FC);
    denom = DC_MAX - DC_MIN;
    int32_t theta = QDIV(num, denom);

    if (theta < 0) 
        theta = 0;
    else if (theta > (UNITS_FC - QONE))
        theta = UNITS_FC - QONE;

    if ((theta < Q2_MIN) && (thetaPre_ > Q3_MAX))
        turns_++;
    else if ((thetaPre_ < Q2_MIN) && (theta > Q3_MAX))
        turns_--;

    if (turns_ >= 0)
        angle_ = (turns_ * UNITS_FC) + theta;
    else if (turns_ < 0)
        angle_ = ((turns_ + 1) * UNITS_FC) - (UNITS_FC - theta);

    thetaPre_ = theta;
}

// ISR delegates
void FeedBackServo::isr0() { if (instances[0]) instances[0]->handleFeedback(); }
void FeedBackServo::isr1() { if (instances[1]) instances[1]->handleFeedback(); }
void FeedBackServo::isr2() { if (instances[2]) instances[2]->handleFeedback(); }
void FeedBackServo::isr3() { if (instances[3]) instances[3]->handleFeedback(); }
void FeedBackServo::isr4() { if (instances[4]) instances[4]->handleFeedback(); }
void FeedBackServo::isr5() { if (instances[5]) instances[5]->handleFeedback(); }
