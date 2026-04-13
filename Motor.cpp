#include "Motor.h"
#include "Config.h"

void Motor::begin() {
    pinMode(MOTOR_EN_A, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

    pinMode(MOTOR_EN_B, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);

    stop();
}

void Motor::setSpeeds(int leftSpeed, int rightSpeed) {
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
}

void Motor::stop() {
    setSpeeds(0, 0);
}

void Motor::setLeftMotor(int speed) {
    // Clamp speed to allowed limits
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    if (speed > 0) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_EN_A, speed);
    } else if (speed < 0) {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
        analogWrite(MOTOR_EN_A, -speed);
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_EN_A, 0);
    }
}

void Motor::setRightMotor(int speed) {
    // Clamp speed to allowed limits
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    if (speed > 0) {
        digitalWrite(MOTOR_IN3, HIGH);
        digitalWrite(MOTOR_IN4, LOW);
        analogWrite(MOTOR_EN_B, speed);
    } else if (speed < 0) {
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, HIGH);
        analogWrite(MOTOR_EN_B, -speed);
    } else {
        digitalWrite(MOTOR_IN3, LOW);
        digitalWrite(MOTOR_IN4, LOW);
        analogWrite(MOTOR_EN_B, 0);
    }
}
