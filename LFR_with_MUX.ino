#include <Arduino.h>
#include "Config.h"
#include "Sensor.h"
#include "Motor.h"
#include "PIDController.h"

// State Machine for the Robot
enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_RUNNING
};

RobotState currentState = STATE_IDLE;
PIDController pid(PID_KP, PID_KI, PID_KD);
unsigned long lastLineDetectTime = 0;

// Button debounce tracking
bool calButtonPressed = false;
bool runButtonPressed = false;

void setup() {
    #if DEBUG_ENABLED
    Serial.begin(SERIAL_BAUD);
    Serial.println("Starting i.R.C LFR System...");
    #endif

    pinMode(BTN_CAL_PIN, INPUT_PULLUP);
    pinMode(BTN_RUN_PIN, INPUT_PULLUP);

    Sensor::begin();
    Motor::begin();

    // Ensure motors are completely off at startup
    Motor::stop();
}

void checkButtons() {
    // Read inverted logic since we use INPUT_PULLUP
    bool currentCal = (digitalRead(BTN_CAL_PIN) == LOW);
    bool currentRun = (digitalRead(BTN_RUN_PIN) == LOW);

    // Detect click for Calibrate Button
    if (currentCal && !calButtonPressed) {
        calButtonPressed = true;
        
        if (currentState == STATE_IDLE) {
            currentState = STATE_CALIBRATING;
            Sensor::resetCalibration();
            #if DEBUG_ENABLED
            Serial.println("Entering MANUAL CALIBRATION mode. Sweep sensors over the line by hand.");
            #endif
        } else if (currentState == STATE_CALIBRATING) {
            currentState = STATE_IDLE;
            #if DEBUG_ENABLED
            Serial.println("Calibration Saved. Exiting to IDLE.");
            Sensor::printCalibration();
            #endif
        }
    } else if (!currentCal) {
        calButtonPressed = false;
    }

    // Detect click for Run Button
    if (currentRun && !runButtonPressed) {
        runButtonPressed = true;
        
        if (currentState == STATE_IDLE) {
            currentState = STATE_RUNNING;
            #if DEBUG_ENABLED
            Serial.println("Entering RUN mode!");
            #endif
            lastLineDetectTime = millis();
        } else if (currentState == STATE_RUNNING) {
            // Click run again to emergency stop
            currentState = STATE_IDLE;
            Motor::stop();
            #if DEBUG_ENABLED
            Serial.println("STOPPED.");
            #endif
        }
    } else if (!currentRun) {
        runButtonPressed = false;
    }
}

void executeCalibration() {
    // In manual mode, we just repeatedly sample while the user sweeps the robot by hand.
    // The motors stay off. State stays in STATE_CALIBRATING until button is pressed again.
    Sensor::calibrate();
    Motor::stop(); // Ensure motors are off
    delay(5); // Small delay to prevent locking the CPU
}

void executeRunLogic() {
    Sensor::update();
    TrackState state = Sensor::getTrackState();

    #if DEBUG_ENABLED
    static unsigned long lastActionPrint = 0;
    bool shouldPrint = (millis() - lastActionPrint > 250);
    if (shouldPrint) lastActionPrint = millis();
    #endif

    switch (state) {
        case TRACK_NORMAL: {
            lastLineDetectTime = millis();
            float error = Sensor::getPositionError();
            
            // The PID algorithm must run continuously to be stable. 
            // We calculate the correction here based on current error.
            float correction = pid.calculate(error);

            // Determine base speed. If the error is very small, we can go faster.
            int baseSpeed = SPEED_NORMAL;
            if (abs(error) < 15) {
                baseSpeed = SPEED_STRAIGHT;
            } else if (abs(error) > 30) {
                baseSpeed = SPEED_CURVE;
            }

            int leftSpeed = baseSpeed + correction;
            int rightSpeed = baseSpeed - correction;
            
            Motor::setSpeeds(leftSpeed, rightSpeed);

            #if DEBUG_ENABLED
            if (shouldPrint) {
                Serial.print("Err: "); Serial.print(error);
                Serial.print(" | Corr: "); Serial.print(correction);
                Serial.print(" | L: "); Serial.print(leftSpeed);
                Serial.print(" | R: "); Serial.println(rightSpeed);
            }
            #endif
            break;
        }

        case TRACK_SHARP_LEFT: {
            lastLineDetectTime = millis();
            Motor::setSpeeds(-SPEED_SHARP, SPEED_SHARP);
            pid.reset();

            #if DEBUG_ENABLED
            if (shouldPrint) Serial.println("!!!!! SHARP LEFT");
            #endif
            break;
        }

        case TRACK_SHARP_RIGHT: {
            lastLineDetectTime = millis();
            Motor::setSpeeds(SPEED_SHARP, -SPEED_SHARP);
            pid.reset();

            #if DEBUG_ENABLED
            if (shouldPrint) Serial.println("!!!!! SHARP RIGHT");
            #endif
            break;
        }

        case TRACK_INTERSECTION: {
            lastLineDetectTime = millis();
            Motor::setSpeeds(SPEED_NORMAL, SPEED_NORMAL);

            #if DEBUG_ENABLED
            if (shouldPrint) Serial.println("+++ INTERSECTION +++");
            #endif
            break;
        }

        case TRACK_ALL_BLACK: {
            lastLineDetectTime = millis();
            Motor::setSpeeds(SPEED_CURVE, SPEED_CURVE);

            #if DEBUG_ENABLED
            if (shouldPrint) Serial.println("### ALL BLACK ###");
            #endif
            break;
        }

        case TRACK_LINE_LOST: {
            if (millis() - lastLineDetectTime > LINE_LOST_TIMEOUT) {
                Motor::stop();
                #if DEBUG_ENABLED
                if (shouldPrint) Serial.println("--- LINE LOST (Gave Up) ---");
                #endif
                break;
            }

            if (Sensor::lastKnownError < 0) {
                Motor::setSpeeds(-RECOVERY_SPEED, RECOVERY_SPEED);
            } else {
                Motor::setSpeeds(RECOVERY_SPEED, -RECOVERY_SPEED);
            }

            #if DEBUG_ENABLED
            if (shouldPrint) Serial.println("??? Recovering Line...");
            #endif
            break;
        }
    }
}

void loop() {
    checkButtons();

    if (currentState == STATE_CALIBRATING) {
        executeCalibration();
    } 
    else if (currentState == STATE_RUNNING) {
        executeRunLogic();
    }
    else {
        // STATE_IDLE
        Motor::stop();

        #if DEBUG_ENABLED
        static unsigned long lastIdlePrint = 0;
        if (millis() - lastIdlePrint > 250) { // Print 4 times a second
            lastIdlePrint = millis();
            Sensor::update(); 
            Sensor::printReadings();
            Serial.print("Pos Error: ");
            Serial.println(Sensor::getPositionError());
        }
        #endif

        delay(50);
    }
}
