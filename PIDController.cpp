#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    
    _previousError = 0;
    _integral = 0;
    _integralLimit = 1000.0; // Prevent integral windup
}

float PIDController::calculate(float error) {
    _integral += error;
    
    // Anti-windup
    if (_integral > _integralLimit) _integral = _integralLimit;
    else if (_integral < -_integralLimit) _integral = -_integralLimit;

    float derivative = error - _previousError;
    _previousError = error;

    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);
    return output;
}

void PIDController::reset() {
    _previousError = 0;
    _integral = 0;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
