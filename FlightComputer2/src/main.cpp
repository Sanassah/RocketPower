#include <Arduino.h>
#include "sensors/FlightData.h"

FlightData flightData;
// put function declarations here:
int myFunction(int, int);

const uint32_t LOOP_PERIOD_MS     = 10;    // target loop period -> 100Hz
const uint32_t REPORT_INTERVAL_MS = 5000;  // how often diagnostics print

uint32_t lastPrintMs     = 0;
uint32_t maxLoopTime     = 0;
uint32_t loopStart       = 0;
uint32_t iterationsCount = 0;

void setup() {
  Serial.begin(115200);
  int result = myFunction(2, 3);
}

void loop() {
  loopStart = millis();
  iterationsCount++;

  uint32_t loopDuration = millis() - loopStart;
  maxLoopTime = max(maxLoopTime, loopDuration);   // worst case since the last report

  if (millis() - lastPrintMs >= REPORT_INTERVAL_MS) {
    lastPrintMs = millis();

    float frequency = iterationsCount / (REPORT_INTERVAL_MS / 1000.0f);
    Serial.printf("Max iteration time(ms)=%lu\n", maxLoopTime);
    Serial.printf("Frequency=%.1f\n", frequency);

    flightData.accel.x_g = 1.0f;
  
    maxLoopTime = 0;
    iterationsCount = 0;
  }

  // busy-wait for the remainder of the loop period -- keeps the loop at a fixed rate
  while (millis() - loopStart < LOOP_PERIOD_MS) {
    // do nothing
  }
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
