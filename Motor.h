#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    // Initializes motor pins
    static void begin();

    // Sets the speed of left and right motors
    // Positive values move forward, negative values move backward
    // Speed is clamped to [-MAX_SPEED, MAX_SPEED]
    static void setSpeeds(int leftSpeed, int rightSpeed);

    // Stops both motors
    static void stop();

private:
    static void setLeftMotor(int speed);
    static void setRightMotor(int speed);
};

#endif // MOTOR_H
