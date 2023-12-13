#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "main.h"
#include "IO.h"
#include "bluetooth.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;

extern EventGroupHandle_t main_event_Group;
extern SemaphoreHandle_t xSemaphore_PMS_ready;
extern SemaphoreHandle_t xSemaphore_SCD4X_ready;
extern SemaphoreHandle_t xSemaphore_bluetooth_ready;

extern volatile Flags flag;
extern volatile Signalss signalss;
extern SensorData sensorData;

bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


void bluetooth_Task(void * parameter) // main BLE task for sending data 
{
    for(;;)
    {
        if(xSemaphoreTake(xSemaphore_bluetooth_ready, portMAX_DELAY) )  // if data ready start sending it via bluetooth
        {
            BLE_Setup();

            // notify changed value
            if (deviceConnected) 
            {

                pCharacteristic->setValue(sensorData.co2);
                pCharacteristic->notify();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
        
            }
            // disconnecting
            if (!deviceConnected && oldDeviceConnected) 
            {
                vTaskDelay(500 / portTICK_PERIOD_MS); // give the bluetooth stack the chance to get things ready
                pServer->startAdvertising(); // restart advertising
                Serial.println("start advertising");
                oldDeviceConnected = deviceConnected;
            }
            // connecting
            if (deviceConnected && !oldDeviceConnected) 
            {
            // do stuff here on connecting
                oldDeviceConnected = deviceConnected;
            }          

        }
    }

}


void BLE_Setup()
{
    
  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );                   

  // Create a BLE Descriptor
  
  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("A very interesting variable");
  pCharacteristic->addDescriptor(pDescr);
  
  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  DEBUG_OUT.println("BLUETOOTH_TASK -> Waiting a client connection to notify...");
}
