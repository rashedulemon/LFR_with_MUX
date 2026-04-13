#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    // Initialize with tuning parameters
    PIDController(float kp, float ki, float kd);

    // Calculate PID correction based on the current error
    float calculate(float error);

    // Reset accumulated integral and derivative state
    void reset();

    // Update PID gains on-the-fly (useful for adaptive PID)
    void setGains(float kp, float ki, float kd);

private:
    float _kp;
    float _ki;
    float _kd;

    float _previousError;
    float _integral;
    float _integralLimit;  // Anti-windup clamp
};

#endif // PIDCONTROLLER_H
