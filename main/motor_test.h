#pragma once

#include <Arduino.h>


void motorTestStart();
void motorTestCancel();

// Returns true once the bounded four-direction test has stopped.
bool motorTestTick();
