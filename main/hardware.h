#pragma once

#include <Arduino.h>


// Motor functions
void motorInit();

void moveLFR(
    int16_t leftSpeed,
    int16_t rightSpeed
);

void stopMotors();

// Actively brake both L298N channels. Navigation uses this at the exact
// forward-probe deadline before it changes to a recovery state.
void brakeMotors();


// Sensor functions
void sensorInit();

void readSensors(uint16_t values[]);
