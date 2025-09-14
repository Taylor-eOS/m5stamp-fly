#include "flow.hpp"
#include <Arduino.h>
#include <SPI.h>
#include <Bitcraze_PMW3901.h>
#include "status_transmitter.h"

Bitcraze_PMW3901 flow(12);
bool flow_init_ok = false;

void setup_flow() {
  SPI.begin(44, 43, 14, 12);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  if (!flow.begin()) {
    sendStatusMessage("Flow init fail");
  } else {
    sendStatusMessage("Flow init ok");
    flow_init_ok = true;
  }
}

void loop_flow() {
  if (!flow_init_ok) return;
  int16_t dx, dy;
  flow.readMotionCount(&dx, &dy);
  uint8_t squal = flow.registerRead(0x07);
  if (dx != 0 || dy != 0) {
    sendStatusMessage("dx: " + String(dx) + ", dy: " + String(dy) + ", SQUAL: " + String(squal));
  } else {
    sendStatusMessage("No motion, SQUAL: " + String(squal));
  }
  delay(20);
}

