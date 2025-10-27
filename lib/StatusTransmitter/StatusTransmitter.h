#ifndef STATUS_TRANSMITTER_H
#define STATUS_TRANSMITTER_H

#include <Arduino.h>

void initStatusTransmitter();
void sendStatusMessage(String message);

#endif
