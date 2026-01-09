#include "FeedBackServo.h"

// Sefine feedback signal pin and servo control pin
#define FEEDBACK_PIN 2
#define SERVO_PIN 9
#define NUM_SERVO_TEETH (25)
#define NUM_TURNTABLE_TEETH (90)

const static int32_t kNumServoTeeth = TOFIXEDPOINT(NUM_SERVO_TEETH - 1);
const static int32_t kNumTurntableTeeth = TOFIXEDPOINT(NUM_TURNTABLE_TEETH - 1);
const static int32_t kGearRatio = QDIV(kNumTurntableTeeth, kNumServoTeeth);

// Set feedback signal pin number
FeedBackServo servo = FeedBackServo(FEEDBACK_PIN);

static char buffer[128];
static char readBuffer[128];


void setup()
{
    Serial.begin(115200);

    // Set servo control pin number
    servo.setServoControl(SERVO_PIN);

    // Adjust as needed.
    servo.setKp(/*Kp=*/TOFIXEDPOINT(4.0),
                /*Ki=*/TOFIXEDPOINT(10.0),
                /*Kd=*/TOFIXEDPOINT(0.00));

    servo.setThresholdDeg(TOFIXEDPOINT(4.0));

    servo.setTargetAngle(0);

    snprintf(buffer, sizeof(buffer), "\n\n\nINIT\n");
    Serial.print(buffer);
    snprintf(buffer, sizeof(buffer), "servo_time_millis,turntable_angle_q8,turntable_target_angle_deg_q8,turntable_error_angle_deg_q8\n");
    Serial.print(buffer);

}

void loop()
{
    if (Serial.available() > 0) {
      memset(readBuffer, 0, sizeof(readBuffer));
      int bytesRead = Serial.readBytes(readBuffer, sizeof(readBuffer));
      int newTarget = 0;
      if (sscanf(readBuffer, "%d", &newTarget) > 0) {
          int32_t targetAngle = ((int32_t)(newTarget)) << Q_FIXEDPOINT;
          servo.setTargetAngle(QMUL(targetAngle, kGearRatio));
          //snprintf(buffer, sizeof(buffer), "set target to %d deg\n", newTarget);
          //Serial.print(buffer);
      }
    }
    
    // Calculate whether new target input request meets specified time interval requirement to prevent mistarget
    const unsigned long currentTimeMillis = millis();
    
    // Rotate servo from 0 to 180 (w/ +-2 threshold) using non-blocking.
    servo.update(currentTimeMillis);

    snprintf(buffer, sizeof(buffer), "%"PRIu32",%"PRId32", %"PRId32", %"PRId32"\n", 
        currentTimeMillis,
        QDIV(servo.getAngle(), kGearRatio),
        QMUL(servo.getTargetAngle(), kGearRatio),
        QDIV(servo.getErrorAngle(), kGearRatio));
    Serial.print(buffer);

}
