#include <EEPROM.h>
#include "lunapb.h"

#define MAX_SENSOR_DATA 4
#define MAX_DEVICE_TYPES 2

struct DeviceEEPROMParams
{
  uint32_t id;
  SensorData sensorData[MAX_SENSOR_DATA];
  DeviceType deviceTypes[MAX_DEVICE_TYPES];
  MicrocontrollerType microType;
  RadioId radioId;
};

struct RadioNRF24EEPROMParams
{
  byte nrf24NodeID;
};

struct EEPROMParams
{
  DeviceEEPROMParams deviceEEPROMParams;
  RadioNRF24EEPROMParams radioNRF24EEPROMParams;
};

EEPROMParams eepromParams = {{0}};

void setup() 
{
  Serial.begin(115200);
  Serial.println("Flashing configuration...");
  
  EEPROMParams eepromParams = {{0}};

  int id = 1;
  
  eepromParams.deviceEEPROMParams.id = id;
  eepromParams.deviceEEPROMParams.microType = MicrocontrollerType_MC_ATMEGA328P;
  eepromParams.deviceEEPROMParams.deviceTypes[0] = DeviceType_DT_AC_SENSOR;
  eepromParams.deviceEEPROMParams.deviceTypes[1] = DeviceType_DT_RELAY;
  eepromParams.deviceEEPROMParams.radioId = RadioId_RID_NRF24;

  eepromParams.radioNRF24EEPROMParams.nrf24NodeID = id;

  EEPROM.put(0x0, eepromParams);

  Serial.println("Flash configuration done");
}

void loop() 
{

}
