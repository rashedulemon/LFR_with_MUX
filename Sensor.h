#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

// Track condition detected by the sensor array
enum TrackState {
    TRACK_NORMAL,        
    TRACK_SHARP_LEFT,    
    TRACK_SHARP_RIGHT,   
    TRACK_INTERSECTION,  
    TRACK_LINE_LOST,     
    TRACK_ALL_BLACK      
};

class Sensor {
public:
    static void begin();

    // ----------------------------------------------------
    // Calibration Methods
    // ----------------------------------------------------
    // Resets min/max arrays to defaults before starting calibration
    static void resetCalibration();

    // Updates the min/max arrays based on current readings
    static void calibrate();

    // Normalizes raw reading using the calibration min/max arrays (returns 0-1000)
    static int getNormalized(int index);

    // ----------------------------------------------------
    // Operational Methods
    // ----------------------------------------------------
    static void update();
    static float getPositionError();
    static TrackState getTrackState();
    static int getActiveSensorCount();
    
    // Debugging
    static void printReadings();
    static void printCalibration();

    static float lastKnownError;

private:
    static int rawReadings[14];
    static int sensorMin[14];
    static int sensorMax[14];
    
    static void selectMuxChannel(byte channel);
};

#endif // SENSOR_H
