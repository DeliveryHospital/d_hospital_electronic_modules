#include <Wire.h>
#include <WirePacker.h>
#include <WireSlaveRequest.h>

#define SLV_ADR 0x10
#define MAX_SLAVE_RESPONSE_LENGTH 32

#define OUT_OF_RANGE -1

#define SDA_PIN 27
#define SCL_PIN 14

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
}


void measureDistance(){
  // first create a WirePacker that will assemble a packet
  WirePacker packer;
  // then add data the same way as you would with Wire
  packer.write((byte)'d');
  //packer.write(/*Mandar uma segunda coisa*/)
  // after adding all data you want to send, close the packet
  packer.end();

  // now transmit the packed data
  Wire.beginTransmission(SLV_ADR);
  while(packer.available()){    // write every packet byte
    Wire.write(packer.read());
  }
  Wire.endTransmission();   // stop transmitting
  delay(250); //reading the sensors takes 243 ms
  WireSlaveRequest slaveReq1(Wire, SLV_ADR, MAX_SLAVE_RESPONSE_LENGTH);
  bool succsess = slaveReq1.request();
  if(succsess){
    int i = 1;
    while(0 < slaveReq1.available()){
      
      int16_t dist;
      dist = slaveReq1.read() << 8;
      dist |= slaveReq1.read();
      //dist = (dist*256) + slaveReq1.read();
      
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      if(dist == OUT_OF_RANGE){
        Serial.print("out of range");
      }else{
        Serial.print(dist);
        Serial.println(" mm ");
      }
      i = i + 1;
    }
    Serial.println();
  } 
}


void measureAccel(){
   // first create a WirePacker that will assemble a packet
  WirePacker packer;
   // then add data the same way as you would with Wire
  packer.write('a');
  //packer.write(/*Mandar uma segunda coisa*/)
  // after adding all data you want to send, close the packet
  packer.end();

  // now transmit the packed data
  Wire.beginTransmission(SLV_ADR);
  while(packer.available()){    // write every packet byte
    Wire.write(packer.read());
  }
  Wire.endTransmission();   // stop transmitting
  delay(250);

  WireSlaveRequest slaveReq2(Wire, SLV_ADR, MAX_SLAVE_RESPONSE_LENGTH);
  bool success = slaveReq2.request();
  if(success){
    int16_t x;
    x = slaveReq2.read() << 8;
    x |= slaveReq2.read();
    Serial.print("AcX = "); Serial.print(x);

    x = slaveReq2.read() << 8;
    x |= slaveReq2.read();
    Serial.print(" | AcY = "); Serial.print(x);
    
    x = slaveReq2.read() << 8;
    x |= slaveReq2.read();
    Serial.print(" | AcZ = "); Serial.print(x);
    
    x = slaveReq2.read() << 8;
    x |= slaveReq2.read();
    Serial.print(" | Tmp = "); Serial.print(((double)x)/340+36.56);
    
    x = slaveReq2.read() << 8;
    x |= slaveReq2.read();
    Serial.print(" | GyX = "); Serial.print(x);
    
    x = slaveReq2.read() << 8;
    x |= slaveReq2.read();
    Serial.print(" | GyY = "); Serial.print(x);

    x = slaveReq2.read() << 8;
    x |= slaveReq2.read();
    Serial.print(" | GyZ = "); Serial.print(x);
    Serial.println();

    }else{
      Serial.println("Error requesting accelerometer");
    }
}

void loop() {
  measureDistance();
  Serial.println();
  delay(200);
  
  measureAccel(); 
  Serial.println();   
  delay(200);
  
}
