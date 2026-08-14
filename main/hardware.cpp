#include "hardware.h"
#include "config.h"
#include "motor_guard.h"


// Default: 3 us. Increase only if adjacent MUX channels contaminate readings;
// decrease after confirming stable values with the installed sensor board.
constexpr uint8_t MUX_SETTLE_US = 3;

static MotorDirectionGuardState leftMotorGuard = { 0, true, 0 };
static MotorDirectionGuardState rightMotorGuard = { 0, true, 0 };


// Control one motor.
//
// Positive speed = forward
// Negative speed = reverse
// Zero           = stop
static void setMotor(
    int16_t speed,
    uint8_t enablePin,
    uint8_t input1,
    uint8_t input2
)
{
  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);
  }
  else if (speed < 0)
  {
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH);

    // analogWrite needs a positive PWM value.
    speed = -speed;
  }
  else
  {
    digitalWrite(input1, LOW);
    digitalWrite(input2, LOW);
  }

  analogWrite(enablePin, speed);
}


void motorInit()
{
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();
}


void moveLFR(
    int16_t leftSpeed,
    int16_t rightSpeed
)
{
  const uint32_t now = micros();
  leftSpeed = motorDirectionGuardCommand(leftMotorGuard, leftSpeed, now);
  rightSpeed = motorDirectionGuardCommand(rightMotorGuard, rightSpeed, now);
  setMotor(
      leftSpeed,
      ENA,
      IN1,
      IN2
  );

  setMotor(
      rightSpeed,
      ENB,
      IN3,
      IN4
  );
}


void stopMotors()
{
  moveLFR(0, 0);
}


void brakeMotors()
{
  motorDirectionGuardReset(leftMotorGuard);
  motorDirectionGuardReset(rightMotorGuard);
  // With both bridge inputs HIGH and full enable PWM, the L298N shorts each
  // motor winding and brakes. Normal stopMotors() remains a low-low coast.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}


// Select one MUX channel and read its analog value.
static uint16_t readMuxChannel(uint8_t channel)
{
  digitalWrite(MUX_A, bitRead(channel, 0));
  digitalWrite(MUX_B, bitRead(channel, 1));
  digitalWrite(MUX_C, bitRead(channel, 2));
  digitalWrite(MUX_D, bitRead(channel, 3));

  // The ADC conversion itself adds acquisition time; one short address-settle
  // interval and one conversion keep the complete 14-channel frame current.
  delayMicroseconds(MUX_SETTLE_US);
  return analogRead(MUX_SIG);
}


void sensorInit()
{
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);
  pinMode(MUX_D, OUTPUT);

  pinMode(MUX_SIG, INPUT);

  // Start with MUX channel zero selected.
  digitalWrite(MUX_A, LOW);
  digitalWrite(MUX_B, LOW);
  digitalWrite(MUX_C, LOW);
  digitalWrite(MUX_D, LOW);
}


void readSensors(uint16_t values[])
{
  // values[0]  contains S0.
  // values[13] contains S13.
  for (
      uint8_t channel = 0;
      channel < SENSOR_COUNT;
      channel++
  )
  {
    values[channel] =
        readMuxChannel(channel);
  }
}
