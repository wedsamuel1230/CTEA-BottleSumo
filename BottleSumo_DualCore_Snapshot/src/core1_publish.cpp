#include "../include/sensor_shared.h"
#include <Arduino.h>

// Memory barrier (portable)
static inline void memBarrier() {
  __asm volatile("" ::: "memory");
}

static uint32_t localSequence = 0;

void publishSensorSnapshot(const SensorSnapshot& draft) {
  uint8_t current = g_sensorExchange.publishIndex;
  uint8_t next = current ^ 1;  // flip 0<->1

  // Copy into the non-published buffer
  g_sensorExchange.buffers[next] = draft;

  // Assign new sequence AFTER data copied
  localSequence++;
  g_sensorExchange.buffers[next].sequence = localSequence;
  g_sensorExchange.latestSequence = localSequence;

  // Ensure all writes are visible before index swap
  memBarrier();

  // Publish
  g_sensorExchange.publishIndex = next;
  // Optional additional barrier if extreme rigor required
  memBarrier();
}
