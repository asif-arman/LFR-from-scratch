#include "settings.h"

#include <EEPROM.h>
#include <avr/pgmspace.h>


uint8_t kpX100 = 30;
uint8_t kdX100 = 8;
uint8_t baseSpeed = 160;
uint16_t sensorThreshold = 400;
uint8_t routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
bool sensorCalibrationValid = false;
uint16_t sensorThresholds[SENSOR_COUNT];


constexpr uint16_t EEPROM_MAGIC = 0x4C46; // "LF"
constexpr uint8_t EEPROM_VERSION = 2;
constexpr uint8_t CALIBRATION_VALID_MASK = 1 << 0;
constexpr uint8_t TUNING_VERSION_SHIFT = 1;
constexpr uint8_t TUNING_VERSION_MASK = 0x0E;
constexpr uint8_t CURRENT_TUNING_VERSION = 1;


// EEPROM byte layout (packed, 39 bytes total):
//  0..1  magic, 2 version, 3 KP, 4 KD, 5 speed,
//  6..7  global threshold, 8 route priority, 9 flags,
// 10..37 fourteen uint16_t calibration thresholds, 38 CRC-8.
struct __attribute__((packed)) SettingsRecord
{
  uint16_t magic;
  uint8_t version;
  uint8_t kp;
  uint8_t kd;
  uint8_t speed;
  uint16_t threshold;
  uint8_t priority;
  uint8_t flags;
  uint16_t calibratedThresholds[SENSOR_COUNT];
  uint8_t crc;
};


static_assert(sizeof(SettingsRecord) == 39, "Unexpected EEPROM layout");


static const uint8_t PRIORITY_TABLE[ROUTE_PRIORITY_COUNT][3] PROGMEM =
{
  { ROUTE_STRAIGHT, ROUTE_LEFT, ROUTE_RIGHT },
  { ROUTE_STRAIGHT, ROUTE_RIGHT, ROUTE_LEFT },
  { ROUTE_LEFT, ROUTE_STRAIGHT, ROUTE_RIGHT },
  { ROUTE_LEFT, ROUTE_RIGHT, ROUTE_STRAIGHT },
  { ROUTE_RIGHT, ROUTE_STRAIGHT, ROUTE_LEFT },
  { ROUTE_RIGHT, ROUTE_LEFT, ROUTE_STRAIGHT }
};


static uint8_t calculateCrc(const SettingsRecord &record)
{
  const uint8_t *bytes =
      reinterpret_cast<const uint8_t *>(&record);
  uint8_t crc = 0xA7;

  for (uint8_t i = 0; i < sizeof(SettingsRecord) - 1; i++)
  {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) :
                           (uint8_t)(crc << 1);
    }
  }

  return crc;
}


static void writeRecord(SettingsRecord &record)
{
  record.crc = calculateCrc(record);
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  for (uint8_t address = 0; address < sizeof(SettingsRecord); address++)
    EEPROM.update(address, bytes[address]);
}


static void restoreDefaults()
{
  kpX100 = 30;
  kdX100 = 8;
  baseSpeed = 160;
  sensorThreshold = 400;
  routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
  sensorCalibrationValid = false;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorThresholds[i] = sensorThreshold;
  }
}


static bool recordIsValid(const SettingsRecord &record)
{
  if (record.magic != EEPROM_MAGIC ||
      record.version != EEPROM_VERSION ||
      record.crc != calculateCrc(record) ||
      record.kp > 200 ||
      record.kd > 200 ||
      record.threshold > 1023 ||
      record.priority >= ROUTE_PRIORITY_COUNT)
  {
    return false;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (record.calibratedThresholds[i] > 1023)
    {
      return false;
    }
  }

  return true;
}


void settingsLoad()
{
  SettingsRecord record;
  EEPROM.get(0, record);

  if (!recordIsValid(record))
  {
    restoreDefaults();
    return;
  }

  kpX100 = record.kp;
  kdX100 = record.kd;
  baseSpeed = record.speed;
  sensorThreshold = record.threshold;
  routePriority = record.priority;
  sensorCalibrationValid =
      (record.flags & CALIBRATION_VALID_MASK) != 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorThresholds[i] = record.calibratedThresholds[i];
  }

  const uint8_t savedTuningVersion =
      (record.flags & TUNING_VERSION_MASK) >> TUNING_VERSION_SHIFT;
  if (savedTuningVersion < CURRENT_TUNING_VERSION)
  {
    // One-time tuning migration: preserve calibration, route policy,
    // thresholds and record layout; replace only the unstable motion tuning.
    kpX100 = 30;
    kdX100 = 8;
    baseSpeed = 160;
    record.kp = kpX100;
    record.kd = kdX100;
    record.speed = baseSpeed;
    record.flags = (record.flags & ~TUNING_VERSION_MASK) |
        (CURRENT_TUNING_VERSION << TUNING_VERSION_SHIFT);
    writeRecord(record);
  }
}


void settingsSaveIfChanged()
{
  SettingsRecord record;
  record.magic = EEPROM_MAGIC;
  record.version = EEPROM_VERSION;
  record.kp = kpX100;
  record.kd = kdX100;
  record.speed = baseSpeed;
  record.threshold = sensorThreshold;
  record.priority = routePriority;
  record.flags = (sensorCalibrationValid ? CALIBRATION_VALID_MASK : 0) |
      (CURRENT_TUNING_VERSION << TUNING_VERSION_SHIFT);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    record.calibratedThresholds[i] = sensorThresholds[i];
  }

  writeRecord(record);
}


void settingsUseGlobalThreshold()
{
  sensorCalibrationValid = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorThresholds[i] = sensorThreshold;
  }
}


void settingsApplyCalibration(const uint16_t thresholds[])
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorThresholds[i] = min(thresholds[i], (uint16_t)1023);
  }

  sensorCalibrationValid = true;
  settingsSaveIfChanged();
}


uint16_t settingsThresholdForSensor(uint8_t sensorIndex)
{
  return sensorCalibrationValid ? sensorThresholds[sensorIndex] :
                                  sensorThreshold;
}


RouteDirection settingsPriorityAt(uint8_t index)
{
  if (routePriority >= ROUTE_PRIORITY_COUNT)
  {
    routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
  }

  if (index >= 3)
  {
    return ROUTE_NONE;
  }

  return static_cast<RouteDirection>(
      pgm_read_byte(&PRIORITY_TABLE[routePriority][index]));
}
