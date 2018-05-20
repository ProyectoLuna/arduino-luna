#include <EEPROM.h>
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include "lunapb.h"

#define MAX_SENSOR_DATA 4
#define MAX_DEVICE_TYPES 2
#define MAX_DEVICE_CONFIG 4
#define MAX_SENSOR_COMMANDS 2

uint32_t millisTimer=0;
char buff[80] = {0};
//uint32_t transactionid = 0;

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

class DeviceConfig
{
public:
  DeviceConfig();
  virtual bool getData(SensorData *data);
  virtual bool execCommand(SensorCommand *cmd);
};

DeviceConfig::DeviceConfig()
{
  
}

bool DeviceConfig::getData(SensorData *data)
{
  return true;
}

bool DeviceConfig::execCommand(SensorCommand *cmd)
{
  return true;
}

class Atmega328PACSensor : public DeviceConfig
{
public:
  Atmega328PACSensor();
  bool getData(SensorData *data) override;
protected:
  int getMaxValue();
};

Atmega328PACSensor::Atmega328PACSensor() : DeviceConfig()
{
  
}

int Atmega328PACSensor::getMaxValue()
{
  int analogPin = 0;
  int sensorValue; //value read from the sensor
  int sensorMax = 0;
  uint32_t start_time = millis();
  while((millis() - start_time) < 400)//sample for t ms
  {
    sensorValue = analogRead(analogPin);
    if (sensorValue > sensorMax)
    {
      sensorMax = sensorValue;
    }
  }
  return sensorMax;
}

bool Atmega328PACSensor::getData(SensorData *data)
{
  int sensor_max;
  float amplitude_current;
  float effective_value;
 
  sensor_max = getMaxValue();
  amplitude_current=(float)sensor_max/1024*5/200*1000000;
  effective_value=amplitude_current/1.414;
  data->unit = SensorUnits_SU_MA;
  data->value = effective_value;
  //Serial.println("mA ef: ");
  //Serial.println(effective_value,1);
  
  return true;
}

class Atmega328PRelay : public DeviceConfig
{
public:
  Atmega328PRelay();
  bool getData(SensorData *data) override;
  bool execCommand(SensorCommand *cmd) override;
private:
  volatile int _relayState;
};

Atmega328PRelay::Atmega328PRelay() : DeviceConfig()
{
  _relayState = HIGH;
  pinMode(2, OUTPUT);
  digitalWrite(2, _relayState);
}

bool Atmega328PRelay::getData(SensorData *data)
{
  data->unit = SensorUnits_SU_RELAYSTATUS;
  data->value = !_relayState;
  return true;  
}

bool Atmega328PRelay::execCommand(SensorCommand *cmd)
{
  switch (cmd->command)
  {
    case SensorCommandType_SCT_TOGGLE:
    {
      if (_relayState == LOW)
      {
        _relayState = HIGH;
        digitalWrite(2, _relayState);
      }
      else
      {
        _relayState = LOW;
        digitalWrite(2, _relayState);
      }
      break;
    }
    default:
      break;
  }
  return true;
}

DeviceConfig* availableConfigurations[_MicrocontrollerType_MAX][_DeviceType_MAX] = {{0}};

class Message
{
public:
  Message(char *buffer = NULL, size_t len = 0);
  char *getBuffer();
  size_t getLength();
private:
  char *_buffer;
  size_t _len;
};

Message::Message(char *buffer, size_t len)
{
  _buffer = buffer;
  _len = len;
}

char *Message::getBuffer()
{
  return _buffer;
}

size_t Message::getLength()
{
  return _len;
}

class Radio
{
public:
  virtual Radio();
  virtual void updateStatus();
  virtual Message receive();
  virtual bool send(Message *message);
  virtual RadioId getRadioId();
protected:
  RadioId _radioId;
};

Radio::Radio()
{
  _radioId = RadioId_RID_UNKNOWN;
}

void Radio::updateStatus()
{
  
}

Message Radio::receive()
{
  return Message();
}

bool Radio::send(Message *message)
{
  return true;
}

RadioId Radio::getRadioId()
{
  return _radioId;
}

class RadioNRF24 : public Radio
{
public:
  RadioNRF24();
  ~RadioNRF24();
  void updateStatus() override;
  Message receive() override;
  bool send(Message *message) override;
  byte getMasterNode();
private:
  RadioNRF24EEPROMParams _params;
  RF24 *_nrf24;
  RF24Network *_nrf24Network;
  RF24Mesh *_nrf24Mesh;
  byte _masterNode;
};

RadioNRF24::RadioNRF24() : Radio()
{
  _nrf24 = new RF24(7, 8);
  _nrf24Network = new RF24Network(*_nrf24);
  _nrf24Mesh = new RF24Mesh(*_nrf24, *_nrf24Network);
  
  _radioId = RadioId_RID_NRF24;
  _masterNode = 0;
  memcpy(&_params, &eepromParams.radioNRF24EEPROMParams, sizeof(RadioNRF24EEPROMParams));

  _nrf24Mesh->setNodeID(_params.nrf24NodeID); 
  _nrf24Mesh->begin();
}

RadioNRF24::~RadioNRF24()
{
  delete _nrf24Mesh;
  delete _nrf24Network;
  delete _nrf24Mesh;
}

void RadioNRF24::updateStatus()
{
  _nrf24Mesh->update();
}

Message RadioNRF24::receive()
{
  if(_nrf24Network->available())
  {
    RF24NetworkHeader header;
    size_t msgLength = _nrf24Network->read(header, buff, sizeof(buff));
    // Serial.print("Rx: ");
    
    int _ID = _nrf24Mesh->getNodeID(header.from_node);
    if( _ID >= 0)
    {
      // Serial.println(_ID);
      return Message(buff, msgLength);
    }
    else
    {
      return Message();
    }
  }
  
  return Message();
}

byte RadioNRF24::getMasterNode()
{
  return _masterNode;
}

bool RadioNRF24::send(Message *msg)
{
  if(!_nrf24Mesh->write(msg->getBuffer(),'M', msg->getLength(), _masterNode))
  {
    if(!_nrf24Mesh->checkConnection())
    {
      // Serial.println("Renew Addr"); // DEBUG
      _nrf24Mesh->renewAddress(); 
      return false;
    }
    else
    {
      //Serial.println("Send fail"); // DEBUG
      return false;
    }
  }
  return true;
}

class Device 
{
public:
  Device();
  bool update();
  bool execCommand(Message *message);
  Message genMessage();
private:
  DeviceEEPROMParams _params;
  DeviceConfig *_config[MAX_DEVICE_CONFIG];
};

Device::Device()
{
  memset(_config, 0x0, sizeof(DeviceConfig *) * MAX_DEVICE_CONFIG);
  memcpy(&_params, &eepromParams.deviceEEPROMParams, sizeof(DeviceEEPROMParams));
  
  for (int i = 0; i < MAX_DEVICE_TYPES && _params.deviceTypes[i]; i++)
  {
    _config[i] = availableConfigurations[_params.microType][_params.deviceTypes[i]];
  }
}

bool Device::update()
{
  for (int i = 0; i < MAX_DEVICE_CONFIG && _config[i]; ++i)
  {
    _config[i]->getData(&_params.sensorData[i]);
  }
  
  return true;
}

RepeatedDevData repeatedData;
RemoteDevData devData[MAX_SENSOR_DATA];
RemoteDevMessage message;
pb_istream_t istream;
pb_ostream_t stream;

bool Device::execCommand(Message *inmessage)
{
  char *rxbuff = inmessage->getBuffer();
    
  message = RemoteDevMessage_init_zero;
  istream = pb_istream_from_buffer(rxbuff, inmessage->getLength());
  
  repeatedData.num = 0;
  repeatedData.data = devData;

  message.data.funcs.decode = &decode_devdata;
  message.data.arg = &repeatedData;

  if (!pb_decode(&istream, RemoteDevMessage_fields, &message))
  {
    // Serial.print("Deco fail "); // DEBUG
    // Serial.println(PB_GET_ERROR(&stream)); // DEBUG
    return false;
  }

  for (int i = 0; i < repeatedData.num; ++i)
  {
    if (!repeatedData.data[i].has_sensor_command)
    {
      continue;
    }
    for (int j = 0; _config[j]; ++j)
    {  
      _config[j]->execCommand(&repeatedData.data[i].sensor_command);
    }
  }
  return true;
}

byte paramToSend = 0;

Message Device::genMessage()
{
  message = RemoteDevMessage_init_zero;

  stream = pb_ostream_from_buffer(buff, sizeof(buff));

  message.header.transaction_id = 0;
  message.header.unique_id.radio_id = _params.radioId;
  message.header.unique_id.id32 = _params.id;

  int numSensorData = 0;
  for (int i = 0; i < MAX_SENSOR_DATA; ++i)
  {
    if (_params.sensorData[i].unit)
    {
      numSensorData++;
    }
  }

  //RemoteDevData devData[MAX_SENSOR_DATA];
  //for (int i = 0; i < MAX_SENSOR_DATA; ++i)
  //for (int i = 0; i < 1; ++i)
  if (paramToSend == 0)
  {
    paramToSend = 1;
    devData[0].has_sensor_data = true;
    devData[0].sensor_data = _params.sensorData[paramToSend];
  }
  else 
  {
    paramToSend = 0;
    devData[0].has_sensor_data = true;
    devData[0].sensor_data = _params.sensorData[paramToSend];
  }
  
  repeatedData.data = devData;
  //repeatedData.num = numSensorData;
  repeatedData.num = 1;

  message.data.funcs.encode = &encode_repeated_devdata;
  message.data.arg = &repeatedData;

  if (!pb_encode(&stream, RemoteDevMessage_fields, &message))
  {
    // Serial.println("Enco fail"); // DEBUG
    return Message();
  }

  return Message(buff, stream.bytes_written);
}

Atmega328PACSensor atmega328PACSensor;
Atmega328PRelay atmega328PRelay;

Device *device;
Radio *radio;

void setup() 
{
  Serial.begin(115200);
  // Serial.println("setup"); // DEBUG
  
  availableConfigurations[MicrocontrollerType_MC_ATMEGA328P][DeviceType_DT_AC_SENSOR] = &atmega328PACSensor;
  availableConfigurations[MicrocontrollerType_MC_ATMEGA328P][DeviceType_DT_RELAY] = &atmega328PRelay;

  EEPROM.get(0x0, eepromParams);
  device = new Device();

  switch (eepromParams.deviceEEPROMParams.radioId)
  {
    case RadioId_RID_NRF24:
    {
      radio = new RadioNRF24();
      break;
    }
    default:
    {
      radio = new Radio();
      break;
    }
  }
}

Message message2;
void loop() 
{
  radio->updateStatus();
  
  message2 = radio->receive();
  if (message2.getBuffer())
  {
    device->execCommand(&message2);
    millisTimer = 0; // Force send
  }

  if(millis() - millisTimer >= 1000)
  {
    device->update();
    message2 = device->genMessage();
    radio->send(&message2);
    
    millisTimer = millis();
  }
}
