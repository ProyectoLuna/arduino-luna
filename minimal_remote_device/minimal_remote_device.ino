#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include "lunapb.h"

RF24 radio(7,8);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

#define otherNodeID 0
byte nodeID = 1;
uint8_t buff2[60];
uint32_t millisTimer=0;
int relayPin = 2;
volatile int relayState = HIGH;

bool execCommand(SensorCommand *cmd)
{
  switch (cmd->command)
  {
    case SensorCommandType_SCT_TOGGLE:
    {
      if (relayState == LOW)
      {
        relayState = HIGH;
        digitalWrite(relayPin, relayState);
      }
      else
      {
        relayState = LOW;
        digitalWrite(relayPin, relayState);
      }
      break;
    }
    default:
      break;
  }
  return true;
}
int getMaxValue()
{
  int analogPin = 0;
  int sensorValue; //value read from the sensor
  int sensorMax = 0;
  uint32_t start_time = millis();
  while((millis() - start_time) < 1000)//sample for 1000ms
  {
    sensorValue = analogRead(analogPin);
    if (sensorValue > sensorMax)
    {
      sensorMax = sensorValue;
    }
  }
  return sensorMax;
}

bool getDataACSensor(SensorData *data)
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

bool getDataRelay(SensorData *data)
{
  data->unit = SensorUnits_SU_RELAYSTATUS;
  data->value = !relayState;
  return true;  
}

void setup() {
  Serial.begin(115200);
  
  pinMode(relayPin, OUTPUT);
  mesh.setNodeID(nodeID); 
  mesh.begin();
}


size_t message_length;

RF24NetworkHeader header;
bool status;
RemoteDevMessage message;
RepeatedDevData repeatedData;
RemoteDevData devData[6];
pb_ostream_t stream;
int len;
pb_istream_t streamIn;

int paramToSend = 0;

void loop() {
  mesh.update();
  if(network.available()){
    len = network.read(header,buff2,sizeof(buff2));
    Serial.println("RX!! ");
//    int _ID = mesh.getNodeID(header.from_node);
//    if( _ID >= 0)
//    {
//       Serial.println(_ID);
//    }
    message = RemoteDevMessage_init_zero;
    streamIn = pb_istream_from_buffer(buff2, len);
    
    repeatedData.num = 0;
    repeatedData.data = devData;
  
    message.data.funcs.decode = &decode_devdata;
    message.data.arg = &repeatedData;
    
    status = pb_decode(&streamIn, RemoteDevMessage_fields, &message);
  
    if (!status)
    {
      Serial.print("Deco fail ");
      //Serial.println(PB_GET_ERROR(&stream));
    }
    else 
    {
      execCommand(&devData[0].sensor_command);
      millisTimer = 0;  
    }
  }

  if(millis() - millisTimer >= 1000)
  {
    message = RemoteDevMessage_init_zero;

    stream = pb_ostream_from_buffer(buff2, sizeof(buff2));

    message.header.transaction_id = 0;
    message.header.unique_id.radio_id = RadioId_RID_NRF24;
    message.header.unique_id.id32 = nodeID;

    if (paramToSend == 0)
    {
      paramToSend = 1;
      devData[0].has_sensor_data = true;
      getDataACSensor(&devData[0].sensor_data);      
    }
    else
    {
      paramToSend = 0;
      devData[0].has_sensor_data = true;
      getDataRelay(&devData[0].sensor_data);
    }

    repeatedData.data = devData;
    repeatedData.num = 1;

    message.data.funcs.encode = &encode_repeated_devdata;
    message.data.arg = &repeatedData;

    status = pb_encode(&stream, RemoteDevMessage_fields, &message);
    message_length = stream.bytes_written;

    if (!status)
    {
        Serial.println("Err");
        //Serial.println(PB_GET_ERROR(&stream));
    }
    // Send an 'M' type to other Node containing the current millis()
    //if(!mesh.write(&millisTimer,'M',sizeof(millisTimer),otherNodeID)){
    if(!mesh.write(buff2,'M', stream.bytes_written,otherNodeID))
    {
      if(!mesh.checkConnection() )
      {
        mesh.renewAddress(); 
      }
    }
    millisTimer = millis();
  }
}
