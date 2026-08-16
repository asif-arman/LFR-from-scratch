#pragma once

#include <Arduino.h>


// Motor functions
void motorInit();

void moveLFR(
    int16_t leftSpeed,
    int16_t rightSpeed
);

void stopMotors();

// Actively brake both L298N channels. Navigation uses this for one control
// tick when a pivot finds its outgoing line, removing rotational momentum.
void brakeMotors();


// Sensor functions
void sensorInit();

void readSensors(uint16_t values[]);
