#include "bluetooth.h"

// function used to configure  bluetooth on ESP32
void bluetoothConfig(BluetoothSerial &SerialBT)
{
    SerialBT.begin("WatherStation");



}


//function used to calculate checksum for BL transfer
uint8_t calculateChecksum(const uint8_t* data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) 
    {
    checksum ^= data[i];
    }
    return checksum;
}


// function used to transmit data via bluetooth 
/*
uint8_t bluetoothTransmit(BluetoothSerial &SerialBT, SensorData &data)
{

    // getting data size 
    size_t data_size = sizeof(data);
    // convert data structure to bytes table 
    uint8_t* data_to_transfer = reinterpret_cast<uint8_t*>(&data);
    //calculate and add checksum to data 
    uint8_t checksum = calculateChecksum(data_to_transfer, data_size);
    

    // check if BT device is available 
    if (SerialBT.available())
    {
        SerialBT.write(data_to_transfer, data_size);
        delay(100);
        SerialBT.write(checksum);

        return 0;
    }
    else 
    {
        // no device available 
        return 1;
     
    }
    
}
*/

