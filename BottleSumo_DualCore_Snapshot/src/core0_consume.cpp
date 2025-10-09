#include "../include/sensor_shared.h"
#include <Arduino.h>

static uint32_t lastSeq = 0;

bool fetchLatestSnapshot(SensorSnapshot& out) {
  uint8_t idx1 = g_sensorExchange.publishIndex;
  const SensorSnapshot* snapPtr = &g_sensorExchange.buffers[idx1];
  // Copy out
  out = *snapPtr;
  // Re-read index to detect race (extremely rare)
  uint8_t idx2 = g_sensorExchange.publishIndex;
  if (idx1 != idx2) {
    // One retry; second copy
    snapPtr = &g_sensorExchange.buffers[idx2];
    out = *snapPtr;
  }
  bool isNew = (out.sequence != lastSeq);
  lastSeq = out.sequence;
  return isNew;
}
