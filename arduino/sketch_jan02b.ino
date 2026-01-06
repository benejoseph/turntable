#include "FeedBackServo.h"

// Sefine feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 9

// Set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

int target = 0;             // State selection
const long kIntervalMillis = 5000; // 2 seconds (in milliseconds)
unsigned long previousTimeMillis = 0;
unsigned long tt = 0;
int milliAccumulator = 0;
int angleAtEpoch = 0;

char buffer[128];

void setup()
{
    Serial.begin(115200);

    // Set servo control pin number
    servo.setServoControl(SERVO_PIN);

    // Adjust as needed.
    servo.setKp(/*Kp=*/TOFIXEDPOINT(2.0),
                /*Ki=*/TOFIXEDPOINT(4.0),
                /*Kd=*/TOFIXEDPOINT(0.00));

    servo.setThresholdDeg(TOFIXEDPOINT(4.0));

    servo.setTargetAngle(0);

}

void loop()
{
    // Calculate whether new target input request meets specified time interval requirement to prevent mistarget
    const unsigned long currentTimeMillis = millis();
    
    // Rotate servo from 0 to 180 (w/ +-2 threshold) using non-blocking.
    servo.update(currentTimeMillis);

    // Retrieve angle from servo
    const int32_t currentAngle = servo.getAngle();

#if 0
    unsigned long dtMillis = currentTimeMillis - tt;
    tt = currentTimeMillis;
    milliAccumulator += dtMillis;
    
    if (milliAccumulator > 100) {
        int deltaAngle = currentAngle - angleAtEpoch;
        angleAtEpoch = currentAngle;
        snprintf(buffer, sizeof(buffer), "%d %d\n", milliAccumulator, deltaAngle);
        Serial.print(buffer);
        milliAccumulator = 0;
    }
#endif

    snprintf(buffer, sizeof(buffer), "%"PRIu32",%"PRId32", %"PRId32", %"PRId32"\n", 
        currentTimeMillis - previousTimeMillis, 
        servo.getTargetAngle(),
        servo.getErrorAngle(), 
        servo.getErrorIntegral());

    Serial.print(buffer);
    //Serial.println(currentAngle, currentTimeMillis);

    if (currentTimeMillis - previousTimeMillis >= kIntervalMillis)
    {
        previousTimeMillis = currentTimeMillis;

        // Prevents improper targetting by providing proper time for relevant calculations to take place
        switch (target)
        {
        case 0:
            target = 1;
            servo.setTargetAngle(0);
            break;
        case 1:
            target = 0;
            servo.setTargetAngle(TOFIXEDPOINT(720));
            break;
        }
    }
}
