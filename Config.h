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
#define PID_KP 0.8
#define PID_KI 0.00001
#define PID_KD 22.0

// ============================================================
// Speed Profiles & Calibration
// ============================================================
#define SPEED_STRAIGHT  150 // Max speeds are between 150-255
#define SPEED_NORMAL    120
#define SPEED_CURVE     90
#define SPEED_SHARP     60

#define CALIBRATION_SPEED 90   // Speed to pivot during auto-calibration
#define CALIBRATION_TIME  3000  // Time (ms) to spin during calibration

#define MIN_SPEED       0
#define MAX_SPEED       255 // ALLOW FULL L298N POWER

// ============================================================
// Advanced Track Thresholds
// ============================================================
#define INTERSECTION_THRESHOLD  10 // Active sensors to classify an intersection
#define SHARP_TURN_THRESHOLD    3  // Active sensors to classify a sharp turn
#define SHARP_TURN_ERROR        40.0

#define LINE_LOST_TIMEOUT       500
#define ALL_BLACK_STOP_TIMEOUT  60 // Decreased to 60ms so it doesn't overshoot the finish box
#define RECOVERY_SPEED          100

// ============================================================
// Debug
// ============================================================
#define DEBUG_ENABLED 1
#define SERIAL_BAUD 115200

#endif // CONFIG_H
