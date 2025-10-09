#include "../include/sensor_shared.h"

SensorSnapshotExchange g_sensorExchange = {
  .buffers = {},
  .publishIndex = 0,
  .latestSequence = 0
};

CommandBlock g_commandBlock = {
  .seq = 0,
  .motorLeft = 0,
  .motorRight = 0,
  .flags = 0,
  .thresholds = {0,0,0,0},
  .thresholdMask = 0
};
