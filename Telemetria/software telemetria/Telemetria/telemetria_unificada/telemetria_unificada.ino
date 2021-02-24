#include "DHT.h" //Sensor temperatura
//#include <Wire.h>
#include <WireSlave.h>

#define SDA_PIN 22
#define SCL_PIN 23
#define SLV_ADR 5

char command;


//Corrente
float vetCorrente1[300];
float valor_medio1;
float valor_Corrente1;
int pin_corrente1=14;
float potencia1;

float vetCorrente2[300];
float valor_medio2;
float valor_Corrente2;
int pin_corrente2=27;
float potencia2;

//Bateria
#define BATERIA_PIN 13
float bateria;

//Temperatura
#define DHTPIN 12
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

float h;
float t;
float f;



void setup()
{
  WireSlave.begin(SDA_PIN, SCL_PIN, SLV_ADR);
  WireSlave.onReceive(recieveEvent);
  WireSlave.onRequest(requestEvent);
  Serial.begin(115200);

  //corrente
  pinMode(pin_corrente1, INPUT);
  pinMode(pin_corrente2, INPUT);

  //Bateria
  dht.begin();

}

void readBattery(){
  bateria = float(analogRead(BATERIA_PIN)*41/4095);
  Serial.print("bateria:");
  Serial.print(bateria);
  Serial.print("[V] \n");
}

void readCurrent(){
  float maior_Valor1 = 0.0;
  float maior_Valor2 = 0.0;
  valor_Corrente1 = 0.0;
  valor_Corrente2 = 0.0;    
 
  float tensao1 = 127.0;
  float tensao2 = 127.0;
  potencia1 = 0.0;
  potencia2 = 0.0;
 
  for(int i = 0; i < 300; i++)
  {
    vetCorrente1[i] = analogRead(pin_corrente1);
    vetCorrente2[i] = analogRead(pin_corrente2);
    delayMicroseconds(600);
  } 
 
  int somaTotal1 = 0;
  int somaTotal2 = 0;
 
  for(int i = 0; i < 300; i++)
  {
    somaTotal1 = vetCorrente1[i] + somaTotal1;
    somaTotal2 = vetCorrente2[i] + somaTotal2;
  }
  valor_medio1 = somaTotal1 / 300;
  valor_medio1 = valor_medio1 * 0.004882812;
  valor_Corrente1 = valor_medio1 - 2.5;
  valor_Corrente1 = valor_Corrente1 * 1000;
  valor_Corrente1 = valor_Corrente1 / 66;       //sensibilidade : 66mV/A para ACS712 30A / 185mV/A para ACS712 5A

  valor_medio2 = somaTotal2 / 300;
  valor_medio2 = valor_medio2 * 0.004882812;
  valor_Corrente2 = valor_medio2 - 2.5;
  valor_Corrente2 = valor_Corrente2 * 1000;
  valor_Corrente2 = valor_Corrente2 / 66;       //sensibilidade : 66mV/A para ACS712 30A / 185mV/A para ACS712 5A
 
  Serial.print("Corrente1 = ");
  Serial.print(valor_Corrente1);
  Serial.println(" A");
  potencia1 = valor_Corrente1 * tensao1;
  Serial.print("Potência1 = ");
  Serial.print(potencia1);
  Serial.println(" W");

  Serial.print("Corrente2 = ");
  Serial.print(valor_Corrente2);
  Serial.println(" A");
  potencia2 = valor_Corrente2 * tensao2;
  Serial.print("Potência2 = ");
  Serial.print(potencia2);
  Serial.println(" W");

  
}

void readTemp(){
  h = dht.readHumidity();
  t = dht.readTemperature();
  f = dht.readTemperature(true);
  
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f); 

}

void sendFloatI2C(float toSend){
  byte byte1, byte2, byte3, byte4;
  unsigned int aux;
  aux = (unsigned int) toSend;
  byte2 = aux;
  byte1 = (aux >> 8);
  toSend -= aux;
  toSend *= 10000;
  aux = (unsigned int) toSend;
  byte4 = aux;
  byte3 = (aux >> 8);
  WireSlave.write(byte1);
  WireSlave.write(byte2);
  WireSlave.write(byte3);
  WireSlave.write(byte4);
}


void loop()
{
  WireSlave.update();
  delay(1);
}

void recieveEvent(int howMany){
  command = WireSlave.read();
   if(command == 'c'){  //Corrente
    readCurrent();
    
  }else if(command == 'b'){  // Bateria
    readBattery();

  }else if(command == 't'){   // Temperatura
    readTemp();
  }
}

void requestEvent(){
  if(command == 'c'){  //Corrente
    sendFloatI2C(valor_Corrente1);
    sendFloatI2C(potencia1);
    sendFloatI2C(valor_Corrente2);
    sendFloatI2C(potencia2);
    
  }else if(command == 'b'){  // Bateria
    sendFloatI2C(bateria);
    
  }else if(command == 't'){   // Temperatura
    sendFloatI2C(h);
    sendFloatI2C(t);
    sendFloatI2C(f);
  }
}
