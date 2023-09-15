#ifndef BLUETOOTH_H
#define BLUETOOTH_H


#include <BluetoothSerial.h>
#include "sensors.h"

void bluetoothConfig(BluetoothSerial &SerialBT);

void bluetoothPrepareData(SensorData &data);

uint8_t calculateChecksum(const uint8_t* data, size_t length);

uint8_t bluetoothTransmit(BluetoothSerial& SerialBT, SensorData& data);

uint8_t bluetoothReceive(BluetoothSerial& SerialBT);










#endif