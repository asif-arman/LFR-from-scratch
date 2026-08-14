#include "settings.h"

#include <EEPROM.h>
#include <avr/pgmspace.h>


uint8_t kpX100 = 30;
uint8_t kdX100 = 8;
uint8_t baseSpeed = 160;
uint16_t sensorThreshold = 400;
uint8_t routePriority = PRIORITY_STRAIGHT_LEFT_RIGHT;
bool boxMode = false;
bool sensorCalibrationValid = false;
uint16_t sensorMinimums[SENSOR_COUNT];
uint16_t sensorMaximums[SENSOR_COUNT];


constexpr uint16_t EEPROM_MAGIC = 0x4C46; // "LF"
constexpr uint8_t EEPROM_VERSION = 3;
constexpr uint8_t CALIBRATION_VALID_MASK = 1 << 0;
constexpr uint8_t BOX_MODE_MASK = 1 << 4;
constexpr uint8_t TUNING_VERSION_SHIFT = 1;
constexpr uint8_t TUNING_VERSION_MASK = 0x0E;
constexpr uint8_t CURRENT_TUNING_VERSION = 1;


// EEPROM byte layout (packed, 67 bytes total):
//  0..1  magic, 2 version, 3 KP, 4 KD, 5 speed,
//  6..7  global threshold, 8 route priority, 9 flags,
// 10..37 fourteen minimums, 38..65 fourteen maximums, 66 CRC-8.
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
  uint16_t calibratedMinimums[SENSOR_COUNT];
  uint16_t calibratedMaximums[SENSOR_COUNT];
  uint8_t crc;
};


static_assert(sizeof(SettingsRecord) == 67, "Unexpected EEPROM layout");


// Version 2 is read only for migration. Midpoint-only calibration cannot
// normalize analog strength, so tuning is retained but calibration is not.
struct __attribute__((packed)) SettingsRecordV2
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


static_assert(sizeof(SettingsRecordV2) == 39, "Unexpected v2 EEPROM layout");


static const uint8_t PRIORITY_TABLE[ROUTE_PRIORITY_COUNT][3] PROGMEM =
{
  { ROUTE_STRAIGHT, ROUTE_LEFT, ROUTE_RIGHT },
  { ROUTE_STRAIGHT, ROUTE_RIGHT, ROUTE_LEFT },
  { ROUTE_LEFT, ROUTE_STRAIGHT, ROUTE_RIGHT },
  { ROUTE_LEFT, ROUTE_RIGHT, ROUTE_STRAIGHT },
  { ROUTE_RIGHT, ROUTE_STRAIGHT, ROUTE_LEFT },
  { ROUTE_RIGHT, ROUTE_LEFT, ROUTE_STRAIGHT }
};


static uint8_t calculateCrc(const uint8_t *bytes, uint8_t length)
{
  uint8_t crc = 0xA7;

  for (uint8_t i = 0; i < length - 1; i++)
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
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  record.crc = calculateCrc(bytes, sizeof(SettingsRecord));
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
  boxMode = false;
  sensorCalibrationValid = false;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorMinimums[i] = 0;
    sensorMaximums[i] = 1023;
  }
}


static bool recordIsValid(const SettingsRecord &record)
{
  if (record.magic != EEPROM_MAGIC ||
      record.version != EEPROM_VERSION ||
      record.crc != calculateCrc(
          reinterpret_cast<const uint8_t *>(&record), sizeof(record)) ||
      record.kp > 200 ||
      record.kd > 200 ||
      record.threshold > 1023 ||
      record.priority >= ROUTE_PRIORITY_COUNT)
  {
    return false;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (record.calibratedMinimums[i] > 1023 ||
        record.calibratedMaximums[i] > 1023 ||
        record.calibratedMinimums[i] >= record.calibratedMaximums[i])
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
    SettingsRecordV2 oldRecord;
    EEPROM.get(0, oldRecord);
    const bool oldValid = oldRecord.magic == EEPROM_MAGIC &&
        oldRecord.version == 2 &&
        oldRecord.crc == calculateCrc(
            reinterpret_cast<const uint8_t *>(&oldRecord),
            sizeof(oldRecord)) &&
        oldRecord.kp <= 200 && oldRecord.kd <= 200 &&
        oldRecord.threshold <= 1023 &&
        oldRecord.priority < ROUTE_PRIORITY_COUNT;

    restoreDefaults();
    if (oldValid)
    {
      kpX100 = oldRecord.kp;
      kdX100 = oldRecord.kd;
      baseSpeed = oldRecord.speed;
      sensorThreshold = oldRecord.threshold;
      routePriority = oldRecord.priority;
      settingsSaveIfChanged();
    }
    return;
  }

  kpX100 = record.kp;
  kdX100 = record.kd;
  baseSpeed = record.speed;
  sensorThreshold = record.threshold;
  routePriority = record.priority;
  boxMode = (record.flags & BOX_MODE_MASK) != 0;
  sensorCalibrationValid =
      (record.flags & CALIBRATION_VALID_MASK) != 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorMinimums[i] = record.calibratedMinimums[i];
    sensorMaximums[i] = record.calibratedMaximums[i];
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
      (boxMode ? BOX_MODE_MASK : 0) |
      (CURRENT_TUNING_VERSION << TUNING_VERSION_SHIFT);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    record.calibratedMinimums[i] = sensorMinimums[i];
    record.calibratedMaximums[i] = sensorMaximums[i];
  }

  writeRecord(record);
}


void settingsUseGlobalThreshold()
{
  sensorCalibrationValid = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorMinimums[i] = 0;
    sensorMaximums[i] = 1023;
  }
}


void settingsApplyCalibration(const uint16_t minimums[],
                              const uint16_t maximums[])
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    sensorMinimums[i] = min(minimums[i], (uint16_t)1023);
    sensorMaximums[i] = min(maximums[i], (uint16_t)1023);
  }

  sensorCalibrationValid = true;
  settingsSaveIfChanged();
}


uint16_t settingsThresholdForSensor(uint8_t sensorIndex)
{
  if (!sensorCalibrationValid) return sensorThreshold;
  return (sensorMinimums[sensorIndex] + sensorMaximums[sensorIndex]) / 2;
}


uint16_t settingsBlackStrength(uint8_t sensorIndex, uint16_t value)
{
  if (!sensorCalibrationValid)
  {
    return value > sensorThreshold ? 1000 : 0;
  }

  const uint16_t minimum = sensorMinimums[sensorIndex];
  const uint16_t maximum = sensorMaximums[sensorIndex];
  if (value <= minimum) return 0;
  if (value >= maximum) return 1000;
  return ((uint32_t)(value - minimum) * 1000UL) / (maximum - minimum);
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
