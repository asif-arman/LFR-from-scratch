#include "buttons.h"
#include "config.h"


// Each switch owns its debounce state, so bounce on one input cannot delay or
// repeat another button. Twenty milliseconds is long enough for typical keys.
constexpr uint16_t DEBOUNCE_MS = 20;
constexpr uint16_t ACTION_LONG_PRESS_MS = 600;


struct ButtonState
{
  bool rawPressed;
  bool stablePressed;
  uint32_t rawChangedAt;
};


enum ButtonEdge : uint8_t
{
  EDGE_NONE,
  EDGE_PRESSED,
  EDGE_RELEASED
};


static ButtonState upState = {};
static ButtonState downState = {};
static ButtonState actionState = {};
static uint32_t actionPressedAt = 0;
static bool actionLongEmitted = false;


static bool pinIsPressed(uint8_t pin)
{
  return digitalRead(pin) == LOW;
}


static void initializeButton(ButtonState &button, uint8_t pin, uint32_t now)
{
  button.rawPressed = pinIsPressed(pin);
  button.stablePressed = button.rawPressed;
  button.rawChangedAt = now;
}


static ButtonEdge updateButton(ButtonState &button,
                               uint8_t pin,
                               uint32_t now)
{
  const bool pressed = pinIsPressed(pin);
  if (pressed != button.rawPressed)
  {
    button.rawPressed = pressed;
    button.rawChangedAt = now;
  }

  if (button.stablePressed == button.rawPressed ||
      (uint32_t)(now - button.rawChangedAt) < DEBOUNCE_MS)
  {
    return EDGE_NONE;
  }

  button.stablePressed = button.rawPressed;
  return button.stablePressed ? EDGE_PRESSED : EDGE_RELEASED;
}


void buttonsInit()
{
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_ACTION_PIN, INPUT_PULLUP);

  const uint32_t now = millis();
  initializeButton(upState, BUTTON_UP_PIN, now);
  initializeButton(downState, BUTTON_DOWN_PIN, now);
  initializeButton(actionState, BUTTON_ACTION_PIN, now);
  actionPressedAt = now;
  actionLongEmitted = actionState.stablePressed;
}


ButtonEvent readButtonEvent()
{
  const uint32_t now = millis();
  const ButtonEdge upEdge = updateButton(upState, BUTTON_UP_PIN, now);
  const ButtonEdge downEdge = updateButton(downState, BUTTON_DOWN_PIN, now);
  const ButtonEdge actionEdge =
      updateButton(actionState, BUTTON_ACTION_PIN, now);

  if (upEdge == EDGE_PRESSED) return BUTTON_UP_CLICK;
  if (downEdge == EDGE_PRESSED) return BUTTON_DOWN_CLICK;

  if (actionEdge == EDGE_PRESSED)
  {
    actionPressedAt = now;
    actionLongEmitted = false;
  }

  // Emit HOLD once while the key is still down. Marking it here suppresses
  // the normal click when that same physical press is later released.
  if (actionState.stablePressed && !actionLongEmitted &&
      (uint32_t)(now - actionPressedAt) >= ACTION_LONG_PRESS_MS)
  {
    actionLongEmitted = true;
    return BUTTON_ACTION_LONG_PRESS;
  }

  if (actionEdge == EDGE_RELEASED && !actionLongEmitted)
  {
    return BUTTON_ACTION_CLICK;
  }

  return BUTTON_NONE;
}
