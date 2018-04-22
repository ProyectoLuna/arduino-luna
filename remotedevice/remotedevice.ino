
 /** RF24Mesh_Example_Node2NodeExtra.ino by TMRh20
  * 
  * This example sketch shows how to communicate between two (non-master) nodes using
  * RF24Mesh & RF24Network
  */
#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>

#include "lunapb.h"

/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(7,8);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

/** 
 * User Configuration: 
 * nodeID - A unique identifier for each radio. Allows addressing to change dynamically
 * with physical changes to the mesh. (numbers 1-255 allowed)
 * 
 * otherNodeID - A unique identifier for the 'other' radio
 * 
 **/
//#define nodeID 2
#define otherNodeID 0 

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int addr = 0;
byte nodeID = 10;
uint32_t millisTimer=0;
uint32_t transactionid = 0;
uint8_t buff[256];

volatile int relayState = HIGH;
volatile int buttonState = HIGH;

const int relayPin = 2;
const int buttonPin = 3;
int analogPin = 0;
float amplitude_current;
float effective_value;
 
void interruptToggle() 
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
}

// EL valor de la potencia efectiva Pe es igual a
// Pe = Ie * Ve * 0.9 
/*Function: Sample for 1000ms and get the maximum value from the SIG pin*/
int getMaxValue()
{
  int sensorValue; //value read from the sensor
  int sensorMax = 0;
  uint32_t start_time = millis();
  while((millis()-start_time) < 1000)//sample for 1000ms
  {
    sensorValue = analogRead(analogPin);
    if (sensorValue > sensorMax)
    {
      /*record the maximum sensor value*/
      sensorMax = sensorValue;
    }
  }
  return sensorMax;
}

void setup() {
  //EEPROM.write(addr, nodeID);
  nodeID = EEPROM.read(addr);
  
  Serial.begin(115200);
  // Set the nodeID
  mesh.setNodeID(nodeID); 
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin();
  
  Serial.print("nodeID: ");
  Serial.print(nodeID, DEC); 
  Serial.println();

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, relayState);

  pinMode(buttonPin, INPUT);
  buttonState = digitalRead(buttonPin);
  //attachInterrupt(digitalPinToInterrupt(buttonPin), interruptToggle, CHANGE);
}


void loop() {
  
  mesh.update();
  
  char reqmsg[16] = {0};

  int currentButtonState = digitalRead(buttonPin);
  if (buttonState != currentButtonState)
  {
    buttonState = currentButtonState;
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
  }
  
  if(network.available()){
        RF24NetworkHeader header;
        //uint32_t mills;
        network.read(header,reqmsg,sizeof(reqmsg));
        Serial.print("Rcv "); Serial.print(reqmsg);
        Serial.print(" from nodeID ");
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
        int _ID = mesh.getNodeID(header.from_node);
        if( _ID >= 0){
           Serial.println(_ID);
        }else{
           Serial.println("Mesh ID Lookup Failed"); 
        }
  }
//  else
//  {
//    Serial.println("Network busy");
//  }
  
  // Send to the other node every second
  if(millis() - millisTimer >= 1000){
    int sensor_max;
    sensor_max = getMaxValue();
    Serial.print("sensor_max = ");
    Serial.println(sensor_max);
    //the VCC on the Grove interface of the sensor is 5v
    amplitude_current=(float)sensor_max/1024*5/200*1000000;
    effective_value=amplitude_current/1.414;
    //Minimum current value can be detected=1/1024*5/200*1000000/1.414=24.4(mA)
    //Only for sinusoidal alternating current
    Serial.println("The amplitude of the current is(in mA)");
    Serial.println(amplitude_current,1);//Only one number after the decimal point
    Serial.println("The effective value of the current is(in mA)");
    Serial.println(effective_value,1);
    millisTimer = millis();

    size_t message_length;
    bool status;
    
    RemoteDevMessage message = RemoteDevMessage_init_zero;

    pb_ostream_t stream = pb_ostream_from_buffer(buff, sizeof(buff));

    message.header.transaction_id = transactionid++;
    message.header.unique_id.radio_id = RadioId_RID_NRF24;
    message.header.unique_id.id32 = nodeID;

    RepeatedSensorData repeatedData;

    SensorData sensorData[4];
    int numData = 2;
    
    sensorData[0].unit = SensorUnits_SU_MAH;
    sensorData[0].value = effective_value;
    sensorData[1].unit = SensorUnits_SU_RELAYSTATUS;
    sensorData[1].value = !relayState;

    repeatedData.data = sensorData;
    repeatedData.num = numData;

    message.data.funcs.encode = &encode_repeated_sensordata;
    message.data.arg = &repeatedData;

    status = pb_encode(&stream, RemoteDevMessage_fields, &message);
    message_length = stream.bytes_written;

    /* Then just check for any errors.. */
    if (!status)
    {
        Serial.println("Encoding failed");
    }
    
    // Send an 'M' type to other Node containing the current millis()
    //if(!mesh.write(&millisTimer,'M',sizeof(millisTimer),otherNodeID)){
    if(!mesh.write(buff,'M', stream.bytes_written,otherNodeID)){
            if( ! mesh.checkConnection() ){
              Serial.println("Renewing Address");
              mesh.renewAddress(); 
            }else{
              Serial.println("Send fail, Test OK"); 
            }
    }
  }
  
}

