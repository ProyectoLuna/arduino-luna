
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
}


void loop() {
  
  mesh.update();
  char reqmsg[16] = {0};

  if(network.available()){
        RF24NetworkHeader header;
        //uint32_t mills;
        network.read(header,reqmsg,sizeof(reqmsg));
        Serial.print("Rcv "); Serial.print(reqmsg);
        Serial.print(" from nodeID ");
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
    int numData = 1;
    for (int i = 0; i < numData; ++i)
    {
        sensorData[i].unit = SensorUnits_SU_MAH;
        sensorData[i].value = millisTimer;
    }

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

