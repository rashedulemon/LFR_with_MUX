#include "Sensor.h"
#include "Config.h"

int Sensor::rawReadings[14];
int Sensor::sensorMin[14];
int Sensor::sensorMax[14];
float Sensor::lastKnownError = 0;

void Sensor::begin() {
    pinMode(MUX_PIN_A, OUTPUT);
    pinMode(MUX_PIN_B, OUTPUT);
    pinMode(MUX_PIN_C, OUTPUT);
    pinMode(MUX_PIN_D, OUTPUT);

    resetCalibration();
}

void Sensor::resetCalibration() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorMin[i] = 1023; // Max possible analog reading
        sensorMax[i] = 0;    // Min possible analog reading
    }
}

void Sensor::update() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        selectMuxChannel(i);
        delayMicroseconds(10);
        rawReadings[i] = analogRead(MUX_SIG_PIN);
    }
}

void Sensor::calibrate() {
    update();
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (rawReadings[i] < sensorMin[i]) {
            sensorMin[i] = rawReadings[i];
        }
        if (rawReadings[i] > sensorMax[i]) {
            sensorMax[i] = rawReadings[i];
        }
    }
}

int Sensor::getNormalized(int index) {
    if (index < 0 || index >= NUM_SENSORS) return 0;
    
    int raw = rawReadings[index];
    int sMin = sensorMin[index];
    int sMax = sensorMax[index];
    
    // Safety check if calibration never ran
    if (sMin >= sMax) return 0;
    
    // Normalize to 0 - 1000
    int normalized = map(raw, sMin, sMax, 0, 1000);
    
    // Clamp values
    if (normalized < 0) normalized = 0;
    if (normalized > 1000) normalized = 1000;
    
    return normalized;
}

float Sensor::getPositionError() {
    long weightedSum = 0;
    long totalValue = 0;
    int activeCount = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        int val = getNormalized(i);

        if (val > SENSOR_THRESHOLD) {
            int weight = 65 - (i * 10);
            weightedSum += (long)weight * val;
            totalValue += val;
            activeCount++;
        }
    }

    if (activeCount == 0) {
        return lastKnownError;
    }

    float error = (float)weightedSum / (float)totalValue;
    lastKnownError = error;
    
    return error;
}

TrackState Sensor::getTrackState() {
    int activeCount = getActiveSensorCount();

    if (activeCount == 0) return TRACK_LINE_LOST;
    if (activeCount >= INTERSECTION_THRESHOLD) return TRACK_ALL_BLACK;
    if (activeCount >= 6) return TRACK_INTERSECTION;

    if (activeCount >= SHARP_TURN_THRESHOLD) {
        float error = getPositionError();
        if (error < -SHARP_TURN_ERROR) return TRACK_SHARP_LEFT;
        if (error > SHARP_TURN_ERROR)  return TRACK_SHARP_RIGHT;
    }

    return TRACK_NORMAL;
}

int Sensor::getActiveSensorCount() {
    int count = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (getNormalized(i) > SENSOR_THRESHOLD) {
            count++;
        }
    }
    return count;
}

void Sensor::printReadings() {
    Serial.print("NORM: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(getNormalized(i));
        Serial.print("\t");
    }
    Serial.println();
}

void Sensor::printCalibration() {
    Serial.println("--- Calibration Data ---");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print("S"); Serial.print(i);
        Serial.print(": Min="); Serial.print(sensorMin[i]);
        Serial.print(" Max="); Serial.println(sensorMax[i]);
    }
    Serial.println("------------------------");
}

void Sensor::selectMuxChannel(byte channel) {
    digitalWrite(MUX_PIN_A, bitRead(channel, 0));
    digitalWrite(MUX_PIN_B, bitRead(channel, 1));
    digitalWrite(MUX_PIN_C, bitRead(channel, 2));
    digitalWrite(MUX_PIN_D, bitRead(channel, 3));
}
