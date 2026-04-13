#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// Push Buttons
// ============================================================
#define BTN_CAL_PIN 5
#define BTN_RUN_PIN 6

// ============================================================
// Multiplexer CD4067BM Pins (14 Array IR Sensor)
// ============================================================
#define MUX_PIN_A   A0
#define MUX_PIN_B   A1
#define MUX_PIN_C   A2
#define MUX_PIN_D   A3
#define MUX_SIG_PIN A6

#define NUM_SENSORS 14

// The normalized threshold (0-1000). 
// Since we now auto-calibrate, 500 represents exactly 50% between the darkest black and brightest white it saw.
#define SENSOR_THRESHOLD 500

// ============================================================
// L298N Motor Driver Pins
// ============================================================
// Left Motor
#define MOTOR_EN_A 10  // Left PWM pin (LPWM)
#define MOTOR_IN1  12  // Left Forward pin (LF)
#define MOTOR_IN2  11  // Left Backward pin (LB)

// Right Motor
#define MOTOR_EN_B 9   // Right PWM pin (RPWM)
#define MOTOR_IN3  8   // Right Forward pin (RF)
#define MOTOR_IN4  7   // Right Backward pin (RB)

// ============================================================
// PID Control Parameters
// ============================================================
// Start with KP = 1.5 and KD = 5.0 for tuning.
// If the robot oscillates slowly, increase KD slightly.
// If it oscillates wildly, your KP is too high.
#define PID_KP 0.7
#define PID_KI 0.0
#define PID_KD 4.0

// ============================================================
// Speed Profiles & Calibration
// ============================================================
// Scaled back down to very slow speeds for visual observation and debugging!
// If the motors hum but don't spin, you may need to tap them or increase slightly.
#define SPEED_STRAIGHT  100
#define SPEED_NORMAL    90
#define SPEED_CURVE     80
#define SPEED_SHARP     75

#define CALIBRATION_SPEED 80   // Speed to pivot during auto-calibration
#define CALIBRATION_TIME  3000 // Time (ms) to spin during calibration

#define MIN_SPEED       0
#define MAX_SPEED       255

// ============================================================
// Advanced Track Thresholds
// ============================================================
#define INTERSECTION_THRESHOLD  10 // Active sensors to classify an intersection
#define SHARP_TURN_THRESHOLD    3  // Active sensors to classify a sharp turn
#define SHARP_TURN_ERROR        40.0

#define LINE_LOST_TIMEOUT       500
#define RECOVERY_SPEED          100

// ============================================================
// Debug
// ============================================================
#define DEBUG_ENABLED 1
#define SERIAL_BAUD 115200

#endif // CONFIG_H
