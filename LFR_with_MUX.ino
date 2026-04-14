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

    static unsigned long allBlackStartTime = 0;
    if (state != TRACK_ALL_BLACK) {
        allBlackStartTime = 0;
    }

    #if DEBUG_ENABLED
    static unsigned long lastActionPrint = 0;
    bool shouldPrint = (millis() - lastActionPrint > 250);
    if (shouldPrint) lastActionPrint = millis();
    #endif

    switch (state) {
        case TRACK_NORMAL: {
            lastLineDetectTime = millis();
            float error = Sensor::getPositionError();

            // Check if the middle two sensors (6 and 7 in a 14-array) are safely on the line
            bool middleOnLine = (Sensor::getNormalized(6) > SENSOR_THRESHOLD && Sensor::getNormalized(7) > SENSOR_THRESHOLD);
            
            int baseSpeed = SPEED_NORMAL;
            float correction = pid.calculate(error);

            if (middleOnLine) {
                // Pin the robot to go perfectly straight without wobbling 
                // if the line is cleanly under the center two sensors.
                baseSpeed = SPEED_STRAIGHT;
                correction = 0; 
                pid.reset(); // Stop derivative windup / bouncing
            }
            else if (abs(error) > 30) {
                baseSpeed = SPEED_CURVE;
            }

            int leftSpeed = baseSpeed + correction;
            int rightSpeed = baseSpeed - correction;
            
            Motor::setSpeeds(leftSpeed, rightSpeed);

            #if DEBUG_ENABLED
            if (shouldPrint) {
                if (middleOnLine) Serial.println("^^^ Going PERFECTLY STRAIGHT (Middle Sensors Dead-Center)");
                else if (error < -15) Serial.println("<<< Correcting LEFT");
                else if (error > 15) Serial.println(">>> Correcting RIGHT");
                else Serial.println("^^^ Going STRAIGHT (Micro-corrections)");
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
             // Forces both left and right wheels to move FORWARD, ignoring errors for '+' crossing
            Motor::setSpeeds(SPEED_NORMAL, SPEED_NORMAL);

            #if DEBUG_ENABLED
            if (shouldPrint) Serial.println("+++ INTERSECTION (+ Crossing) - Going FORWARD +++");
            #endif
            break;
        }

        case TRACK_ALL_BLACK: {
            lastLineDetectTime = millis();
            
            if (allBlackStartTime == 0) {
                allBlackStartTime = millis();
            }

            if (millis() - allBlackStartTime > ALL_BLACK_STOP_TIMEOUT) {
                // Stop the LFR when an all black square is found for the specified timeout
                currentState = STATE_IDLE; 
                Motor::stop();

                #if DEBUG_ENABLED
                Serial.println("### ALL BLACK SQUARE DETECTED. STOPPING LFR ###");
                #endif
            } else {
                // Brake immediately to prevent gliding over and missing the finish box before the timer runs out!
                // If it's a false positive (thick intersection), it will only stutter for 60ms and resume.
                Motor::stop();
            }
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
