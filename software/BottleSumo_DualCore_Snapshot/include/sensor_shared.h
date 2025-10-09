#pragma once
#include <stdint.h>

enum EdgeDirection : uint8_t {
  EDGE_SAFE = 0,
  EDGE_FRONT,
  EDGE_BACK,
  EDGE_LEFT,
  EDGE_RIGHT,
  EDGE_FRONT_LEFT,
  EDGE_FRONT_RIGHT,
  EDGE_BACK_LEFT,
  EDGE_BACK_RIGHT
};

struct SensorSnapshot {
  // Sequencing & timing
  uint32_t sequence;          // Monotonic
  uint32_t captureMillis;     // When IR frame ended
  uint32_t tofMillis;         // Last ToF composite timestamp

  // IR (4 sensors)
  int16_t  irRaw[4];
  float    irVolts[4];

  // Derived edge info
  uint8_t  dangerLevel;       // 0..4
  uint8_t  edgeDetected;      // 0/1
  EdgeDirection edgeDir;      // enum

  // Runtime thresholds used for this evaluation (optional)
  float    thresholds[4];

  // ToF (3 sensors)
  uint16_t tofDist[3];
  uint8_t  tofValidMask;      // bit0=Right, bit1=Front, bit2=Left
  uint8_t  opponentDirMask;   // e.g. bit0=FRONT, bit1=RIGHT, bit2=LEFT

  // Buttons
  uint8_t  buttonsStableMask; // debounced
  uint8_t  buttonsEdgeMask;   // newly changed (1=changed this publish)

  // Health / status flags
  uint16_t statusFlags;       // bits for sensor offline, stale, etc.

  // Reserved for future alignment / expansion
  uint8_t  reserved[6];
};

// Double buffer container
struct SensorSnapshotExchange {
  SensorSnapshot buffers[2];
  volatile uint8_t publishIndex;     // Which buffer is current
  volatile uint32_t latestSequence;  // Mirror for quick staleness check
};

extern SensorSnapshotExchange g_sensorExchange;

// Command channel Core0 -> Core1
struct CommandBlock {
  volatile uint32_t seq;          // increments when updated
  int16_t motorLeft;
  int16_t motorRight;
  uint16_t flags;                 // bit flags for mode toggles
  float    thresholds[4];         // optional threshold override
  uint8_t  thresholdMask;         // bit i set => thresholds[i] valid
  uint8_t  reserved[3];
};

extern CommandBlock g_commandBlock;

// Helper inline (optional)
inline const SensorSnapshot* getLatestSnapshotPtr() {
  return &g_sensorExchange.buffers[g_sensorExchange.publishIndex];
}
