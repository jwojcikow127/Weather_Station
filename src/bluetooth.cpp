#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <string>
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
//BLEDescriptor *pDescr;
//BLE2902 *pBLE2902;

extern EventGroupHandle_t main_event_Group;
extern SemaphoreHandle_t xSemaphore_PMS_ready;
extern SemaphoreHandle_t xSemaphore_SCD4X_ready;
extern SemaphoreHandle_t xSemaphore_bluetooth_ready;

extern volatile Flags flag;
extern volatile Signalss signalss;
extern SensorData sensorData;

bool deviceConnected = false;
bool oldDeviceConnected = false;

//BLECharacteristic* pm1_0_Characteristics = NULL;
//BLEDescriptor* pm1_0_Descriptor;
//BLE2902 *pBLE2902;


uint8_t data_send = 0;

// bluetooth variables define 

BLECharacteristic pm1_0_Characteristics(PM1_UUID,BLECharacteristic::PROPERTY_NOTIFY);
//BLEDescriptor pm1_0_Descriptor(BLEUUID((uint16_t)0x2901));

BLECharacteristic pm2_5_Characteristics(PM2_5_UUID,BLECharacteristic::PROPERTY_NOTIFY);
//BLEDescriptor pm2_5_Descriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic pm10_Characteristics(PM10_UUID,BLECharacteristic::PROPERTY_NOTIFY);
//BLEDescriptor pm10_Descriptor(BLEUUID((uint16_t)0x2903));

BLECharacteristic temperature_Characteristics(TEMP_UUID,BLECharacteristic::PROPERTY_NOTIFY);
//BLEDescriptor temperature_Descriptor(BLEUUID((uint16_t)0x2904));

BLECharacteristic humidity_Characteristics(HUMIDITY_UUID,BLECharacteristic::PROPERTY_NOTIFY);
//BLEDescriptor humidity_Descriptor(BLEUUID((uint16_t)0x2904));

BLECharacteristic Gases_Characteristics(GASES_UUID,BLECharacteristic::PROPERTY_NOTIFY);
//BLEDescriptor CO2_Descriptor(BLEUUID((uint16_t)0x2904));

BLECharacteristic light_Intensity_Characteristics(LIGHT_UUID,BLECharacteristic::PROPERTY_NOTIFY);
//BLEDescriptor light_Intensity_Descriptor(BLEUUID((uint16_t)0x2904));

//BLECharacteristic Confirm_Characteristics(CONFIRM_UUID,BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
//BLEDescriptor Confirm_Descriptor(BLEUUID((uint16_t)0x2910));


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

/*
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *Confirm_Characteristics) {
      std::string value = Confirm_Characteristics->getValue();

      if (value.length() == sizeof(int)) {
        memcpy(&data_send, value.data(), sizeof(int));
        DEBUG_OUT.println("Received value: ");
        DEBUG_OUT.println(data_send);
        // Handle the received value as needed
      }
    }
};
*/

void bluetooth_Task(void * parameter) // main BLE task for sending data 
{

    // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *sensorService = pServer->createService(SERVICE_UUID);
  
  sensorService->addCharacteristic(&pm1_0_Characteristics);

  sensorService->addCharacteristic(&pm2_5_Characteristics);

  sensorService->addCharacteristic(&pm10_Characteristics);
 
  sensorService->addCharacteristic(&Gases_Characteristics);
  
  sensorService->addCharacteristic(&temperature_Characteristics);
 
  sensorService->addCharacteristic(&humidity_Characteristics);
  
  sensorService->addCharacteristic(&light_Intensity_Characteristics);
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);


    for(;;)
    {
        if(xSemaphoreTake(xSemaphore_bluetooth_ready, portMAX_DELAY) )  // if data ready start sending it via bluetooth
        {
            flag.led_state = CONST_FLASH;
            // Start the service
            sensorService->start();
            // start advertising
            
            BLEDevice::startAdvertising();
            
            //pServer->getAdvertising()->start();
            DEBUG_OUT.println("BLUETOOTH_TASK -> Waiting a client connection to notify...");

            // preparing data to be send as strings 
            static char pm1_0[10];
            static char pm2_5[10];
            static char pm10[10];
            static char temperature[10];
            static char humidity[10];
            static char CO2[25];
            static char light_intensity[25]; 
            static char MQ2[10];
            static char IR[10];
           

            
            snprintf(pm1_0,sizeof(pm1_0), "%.d", sensorData.pms_data.PM_AE_UG_1_0);
            strcat(pm1_0," ug/m3");
            snprintf(pm2_5,sizeof(pm2_5), "%.d", sensorData.pms_data.PM_AE_UG_2_5);
            strcat(pm2_5," ug/m3");
            snprintf(pm10,sizeof(pm10), "%.d", sensorData.pms_data.PM_AE_UG_10_0);
            strcat(pm10," ug/m3");
            snprintf(temperature,sizeof(temperature), "%.2f", sensorData.temperature);
            strcat(temperature," C");
            snprintf(humidity,sizeof(humidity), "%.2f", sensorData.humidity);
            strcat(humidity," %RH");
            snprintf(CO2,sizeof(CO2), "%.d", sensorData.co2);
            strcat(CO2," ppm, ");
            snprintf(light_intensity,sizeof(light_intensity), "%.d", sensorData.light_intensity_sensor_data);
            strcat(light_intensity, " lux, ");
            snprintf(MQ2,sizeof(MQ2), "%.d", sensorData.MQ2_sensor_data);
            strcat(MQ2, " %");
            snprintf(IR,sizeof(IR), "%.d", sensorData.ir_sensor_data);
            strcat(IR, " h");
            

            // merge all light params into one char table
            strcat(light_intensity, IR);
            strcat(CO2, MQ2);
            
            while(true)
            {
              if (deviceConnected) 
              {
                  flag.led_state = SENDING_DATA;
                  // if connected in app send data 4 times then break the loop and back to idle mode
                  for(uint8_t i=0; i<3; i++)
                  {
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    pm1_0_Characteristics.setValue(pm1_0);
                    pm1_0_Characteristics.notify();
                    DEBUG_OUT.println(pm1_0);

                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    pm2_5_Characteristics.setValue(pm2_5);
                    pm2_5_Characteristics.notify();
                    DEBUG_OUT.println(pm2_5);

                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    pm10_Characteristics.setValue(pm10);
                    pm10_Characteristics.notify();
                    DEBUG_OUT.println(pm10);
          
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    Gases_Characteristics.setValue(CO2);
                    Gases_Characteristics.notify();
                    DEBUG_OUT.println(CO2);

                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    temperature_Characteristics.setValue(temperature);
                    temperature_Characteristics.notify();
                    DEBUG_OUT.println(temperature);

                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    humidity_Characteristics.setValue(humidity);
                    humidity_Characteristics.notify();
                    DEBUG_OUT.println(humidity);
                  
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                    light_Intensity_Characteristics.setValue(light_intensity);
                    light_Intensity_Characteristics.notify();
                    DEBUG_OUT.println(light_intensity);

                    vTaskDelay(1500 / portTICK_PERIOD_MS);
                    DEBUG_OUT.println("BLUETOOTH TASK -> Sending data");
                  }

                  // memory clear after sending data
                  memset(pm1_0, ' ', sizeof(pm1_0));
                  memset(pm2_5, ' ', sizeof(pm2_5));
                  memset(pm10, ' ', sizeof(pm10));
                  memset(temperature, ' ', sizeof(temperature));
                  memset(CO2, ' ', sizeof(CO2));
                  memset(humidity, ' ', sizeof(humidity));
                  memset(light_intensity, ' ', sizeof(light_intensity));

                  break; 
              
              }
                // disconnecting
              if (!deviceConnected && oldDeviceConnected) 
                {
                    vTaskDelay(500 / portTICK_PERIOD_MS); // give the bluetooth stack the chance to get things ready
                    //pServer->startAdvertising(); // restart advertising
                    pServer->startAdvertising();
                    oldDeviceConnected = deviceConnected;
                    DEBUG_OUT.println("BLUETOOTH TASK -> Not connected");
                }

              // connecting
              if (deviceConnected && !oldDeviceConnected) 
              {
              // do stuff here on connecting
                  oldDeviceConnected = deviceConnected;
                 
                  DEBUG_OUT.println("BLUETOOTH TASK -> Connecting");
              } 


            }

            flag.data_send = 1;
            flag.led_state = SLEEP_MODE;
            BLEDevice::stopAdvertising();
            sensorService->stop();
            

            //BLEDevice::deinit();
            DEBUG_OUT.println("BLUETOOTH TASK -> Suspending this task ");
            DEBUG_OUT.println("MEASURE TASK -> IDLE MODE");
            //vTaskSuspend(NULL);
            
        }
    }

}


