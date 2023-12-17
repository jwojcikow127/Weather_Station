#ifndef BLUETOOTH_H
#define BLUETOOTH_H


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PM1_UUID "c55740ca-99cb-11ee-b9d1-0242ac120002"
#define PM2_5_UUID "c5574462-99cb-11ee-b9d1-0242ac120002"
#define PM10_UUID "c55746ce-99cb-11ee-b9d1-0242ac120002"
#define TEMP_UUID "c55749f8-99cb-11ee-b9d1-0242ac120002"
#define HUMIDITY_UUID "c5574c6e-99cb-11ee-b9d1-0242ac120002"
#define CO2_UUID "c5574f8e-99cb-11ee-b9d1-0242ac120002"
#define LIGHT_UUID "c55751a0-99cb-11ee-b9d1-0242ac120002"
#define MQ2_UUID "c557551a-99cb-11ee-b9d1-0242ac120002"
#define IR_UUID "c5575844-99cb-11ee-b9d1-0242ac120002"
#define CONFIRM_UUID "dcbac5c2-9cd0-11ee-8c90-0242ac120002"


void bluetooth_Task(void * parameter);
void BLE_Setup();



#endif 