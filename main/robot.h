#pragma once

#include <Arduino.h>


// Convert analog values into a compact 14-bit pattern.
uint16_t makeSensorPattern(
    const uint16_t values[]
);


// Find the average position of the active sensors.
int16_t calculateLinePosition(
    uint16_t pattern
);


// Convert the line position into a signed error.
int16_t calculateLineError(
    int16_t linePosition
);


// Calculate the proportional part of PD.
int16_t calculateProportionalCorrection(
    int16_t error
);


// Calculate the derivative part of PD.
int16_t calculateDerivativeCorrection(
    int16_t currentError,
    int16_t previousError
);