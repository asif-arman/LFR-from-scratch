#include "buttons.h"
#include "config.h"


// Default: 25 ms. A button must remain stable this long. Increase if presses
// bounce into duplicates; decrease if the controls feel unresponsive.
constexpr uint16_t DEBOUNCE_MS = 25;


// Default: 350 ms. Maximum LEFT double-click gap. RIGHT deliberately uses a
// single debounced press because it is the primary Enter/Edit/OK control.
constexpr uint16_t DOUBLE_CLICK_MS = 350;


// One bit represents one button.
// This avoids four separate state arrays and saves SRAM.
constexpr uint8_t UP_MASK = 1 << 0;
constexpr uint8_t DOWN_MASK = 1 << 1;
constexpr uint8_t LEFT_MASK = 1 << 2;
constexpr uint8_t RIGHT_MASK = 1 << 3;


static uint8_t rawMask = 0;
static uint8_t stableMask = 0;

static uint32_t rawChangedAt = 0;


// Zero means that no first LEFT click is currently waiting.
static uint32_t firstLeftClickAt = 0;


// Read all four physical buttons into one byte.
//
// Because INPUT_PULLUP is used:
//
// LOW  = pressed
// HIGH = released
static uint8_t readPressedMask()
{
  uint8_t mask = 0;

  if (digitalRead(BUTTON_UP_PIN) == LOW)
  {
    mask |= UP_MASK;
  }

  if (digitalRead(BUTTON_DOWN_PIN) == LOW)
  {
    mask |= DOWN_MASK;
  }

  if (digitalRead(BUTTON_LEFT_PIN) == LOW)
  {
    mask |= LEFT_MASK;
  }

  if (digitalRead(BUTTON_RIGHT_PIN) == LOW)
  {
    mask |= RIGHT_MASK;
  }

  return mask;
}


void buttonsInit()
{
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);

  rawMask = readPressedMask();
  stableMask = rawMask;

  rawChangedAt = millis();
  firstLeftClickAt = 0;
}


ButtonEvent readButtonEvent()
{
  const uint32_t now = millis();

  const uint8_t newRawMask =
      readPressedMask();


  // Remove an unfinished LEFT click when its
  // double-click time expires.
  if (
      firstLeftClickAt != 0 and
      now - firstLeftClickAt > DOUBLE_CLICK_MS
  )
  {
    firstLeftClickAt = 0;
  }


  // Restart the debounce timer whenever the raw
  // electrical reading changes.
  if (newRawMask != rawMask)
  {
    rawMask = newRawMask;

    rawChangedAt = now;
  }


  // The new reading has not remained stable long enough.
  if (now - rawChangedAt < DEBOUNCE_MS)
  {
    return BUTTON_NONE;
  }


  // Nothing changed after debouncing.
  if (stableMask == rawMask)
  {
    return BUTTON_NONE;
  }


  // Only newly pressed buttons create events.
  // Releasing a button creates no event.
  const uint8_t pressed =
      rawMask & ~stableMask;

  stableMask = rawMask;


  // UP and DOWN use normal single clicks.
  if (pressed & UP_MASK)
  {
    return BUTTON_UP_CLICK;
  }

  if (pressed & DOWN_MASK)
  {
    return BUTTON_DOWN_CLICK;
  }


  // LEFT uses double-click.
  if (pressed & LEFT_MASK)
  {
    if (
        firstLeftClickAt != 0 and
        now - firstLeftClickAt <= DOUBLE_CLICK_MS
    )
    {
      firstLeftClickAt = 0;

      return BUTTON_LEFT_DOUBLE_CLICK;
    }

    // This was only the first click.
    firstLeftClickAt = now;
  }


  // RIGHT emits exactly once on its debounced press edge. Holding it cannot
  // repeat because stableMask does not change again until physical release.
  if (pressed & RIGHT_MASK)
  {
    return BUTTON_RIGHT_CLICK;
  }


  return BUTTON_NONE;
}
