#include <Wire.h>
#include <WireSlaveRequest.h>
#include <WirePacker.h>

#define SLV_ADR 0x20
#define MAX_SLAVE_RESPONSE_LENGTH 32

#define SDA_PIN 27
#define SCL_PIN 14



float leFloatI2C(WireSlaveRequest slaveReq){
  byte byte1, byte2, byte3, byte4; 
  unsigned int aux;
  float numero;
  byte1 = slaveReq.read();
  byte2 = slaveReq.read();
  byte3 = slaveReq.read();
  byte4 = slaveReq.read();

  aux = (byte3 <<8) | byte4;
  numero = (float)(aux*0.0001);
  aux = (byte1<<8) | byte2;
  numero += aux;
  return numero;
}

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
}

void leCorrente(){
   // first create a WirePacker that will assemble a packet
  WirePacker packer;
  // then add data the same way as you would with Wire
  packer.write((byte)'c');
  //packer.write(/*Mandar uma segunda coisa*/)
  // after adding all data you want to send, close the packet
  packer.end();
  // now transmit the packed data
  Wire.beginTransmission(SLV_ADR);
  while(packer.available()){    // write every packet byte
    Wire.write(packer.read());
  }
  Wire.endTransmission();   // stop transmitting
  WireSlaveRequest slaveReq(Wire, SLV_ADR, MAX_SLAVE_RESPONSE_LENGTH);
  bool success = slaveReq.request();
  if(success){
    while(0 < slaveReq.available()){
      float corrente1, corrente2, potencia1, potencia2;
      corrente1 = leFloatI2C(slaveReq);
      potencia1 = leFloatI2C(slaveReq);
      corrente2 = leFloatI2C(slaveReq);
      potencia2 = leFloatI2C(slaveReq);

      Serial.print("Corrente 1: ");
      Serial.print(corrente1);
      Serial.println(" A");
      Serial.print("Potencia 1: ");
      Serial.print(potencia1);
      Serial.println(" W");
      Serial.print("Corrente 2: ");
      Serial.println(corrente2);
      Serial.println(" A");
      Serial.print("Potencia 2: ");
      Serial.println(potencia2);
      Serial.println(" W"); 
    }
  }


  
}

void leBateria(){
  // first create a WirePacker that will assemble a packet
  WirePacker packer;
  // then add data the same way as you would with Wire
  packer.write((byte)'b');
  //packer.write(/*Mandar uma segunda coisa*/)
  // after adding all data you want to send, close the packet
  packer.end();
  WireSlaveRequest slaveReq(Wire, SLV_ADR, MAX_SLAVE_RESPONSE_LENGTH);
  bool success = slaveReq.request();
  if(success){
    while(0 < slaveReq.available()){
      float bateria;
      bateria = leFloatI2C(slaveReq);

      Serial.print("Bateria: ");
      Serial.print(bateria);
      Serial.println(" V");
    }
  } 
}

void leTemperatura(){
   // first create a WirePacker that will assemble a packet
  WirePacker packer;
  // then add data the same way as you would with Wire
  packer.write((byte)'t');
  //packer.write(/*Mandar uma segunda coisa*/)
  // after adding all data you want to send, close the packet
  packer.end();
  WireSlaveRequest slaveReq(Wire, SLV_ADR, MAX_SLAVE_RESPONSE_LENGTH);
  bool success = slaveReq.available();
  if(success){
    while(0 < slaveReq.available()){
      float h, t, f;
      h = leFloatI2C(slaveReq);
      t = leFloatI2C(slaveReq);
      f = leFloatI2C(slaveReq);

      Serial.print(F("Humidity: "));
      Serial.print(h);
      Serial.print(F("%  Temperature: "));
      Serial.print(t);
      Serial.print(F("°C "));
      Serial.println(f);
    }
  }
}


void loop() {
  leTemperatura();
  delay(500);
  leCorrente();
  delay(500);
  leBateria();
  delay(500);
}
