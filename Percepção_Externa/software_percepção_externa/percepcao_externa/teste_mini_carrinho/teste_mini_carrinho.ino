// Programa: Acelerometro e sensoriamento de distancia ESP
#include "Adafruit_VL53L0X.h"  // Biblioteca sensor de dist
#include <Wire.h> // Biblioteca de comunicação I2C
#include <WireSlave.h> // Biblioteca de comunicação slave I2C
#include <WirePacker.h>


/*
 * Definições de alguns endereços mais comuns do MPU6050 (multi-sensor acelerometro e temperatura)
 * os registros podem ser facilmente encontrados no mapa de registros do MPU6050
 */
const int MPU_ADDR =      0x68; // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I =      0x75; // registro de identificação do dispositivo
const int PWR_MGMT_1 =    0x6B; // registro de configuração do gerenciamento de energia
const int GYRO_CONFIG =   0x1B; // registro de configuração do giroscópio
const int ACCEL_CONFIG =  0x1C; // registro de configuração do acelerômetro
const int ACCEL_XOUT =    0x3B; // registro de leitura do eixo X do acelerômetro
 
const int sda1_pin = 22; // definição do pino I2C SDA sensores
const int scl1_pin = 23; // definição do pino I2C SCL sensores

const int sda2_pin = 27; // definição do pino I2C SDA controlhe geral
const int scl2_pin = 14; // definição do pino I2C SCL controlhe geral
const int address = 0x10;
char command;

// variáveis para armazenar os dados "crus" do acelerômetro
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; 
// Ac -> aceleracao; Tmp -> Temperatura; Gx -> aceleracao angular

/* ----------------------------------------------------------------------------------*/

#define OUT_OF_RANGE -1

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x31
#define LOX2_ADDRESS 0x32
#define LOX3_ADDRESS 0x33
#define LOX4_ADDRESS 0x34
#define LOX5_ADDRESS 0x35
#define LOX6_ADDRESS 0x36

// set the pins to shutdown
#define SHT_LOX1 4
#define SHT_LOX2 16
#define SHT_LOX3 17
#define SHT_LOX4 5
#define SHT_LOX5 18
#define SHT_LOX6 19


// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
VL53L0X_RangingMeasurementData_t measure5;
VL53L0X_RangingMeasurementData_t measure6;

// this is sent whith the measurments
int16_t measureToSend1;
int16_t measureToSend2;
int16_t measureToSend3;
int16_t measureToSend4;
int16_t measureToSend5;
int16_t measureToSend6;


/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  //digitalWrite(SHT_LOX1, LOW);    
  //digitalWrite(SHT_LOX2, LOW);
  //digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);
  delay(10);
  // all unreset
  //digitalWrite(SHT_LOX1, HIGH);
  //digitalWrite(SHT_LOX2, HIGH);
  //digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);
  digitalWrite(SHT_LOX5, HIGH);
  digitalWrite(SHT_LOX6, HIGH);
  delay(10);

  /*
  // activating LOX1 and reseting all
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2 and reseting all
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);

  // initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX3 and reseting all
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);

  // initing LOX3
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }
  delay(10);
*/
  // activating LOX4 and reseting all
  digitalWrite(SHT_LOX4, HIGH);
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);

  // initing LOX4
  if(!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot fourth VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX5 and reseting all
  digitalWrite(SHT_LOX5, HIGH);
  digitalWrite(SHT_LOX6, LOW);

  // initing LOX5
  if(!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("Failed to boot fifth VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX6 and reseting all
  digitalWrite(SHT_LOX6, HIGH);

  // initing LOX6
  if(!lox6.begin(LOX6_ADDRESS)) {
    Serial.println(F("Failed to boot sixth VL53L0X"));
    while(1);
  }
  delay(10);
  
}




void read_dual_sensors() {
  
  //lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  //lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  //lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
  lox5.rangingTest(&measure5, false); // pass in 'true' to get debug data printout!
  lox6.rangingTest(&measure6, false); // pass in 'true' to get debug data printout!
  /*
  // print sensor one reading
  Serial.print(F("1: "));
  if(measure1.RangeStatus != 4) {     // if not out of range
    Serial.print(measure1.RangeMilliMeter);
    measureToSend1 = measure1.RangeMilliMeter;    
  } else {
    Serial.print(F("Out of range"));
    measureToSend1 = OUT_OF_RANGE;
  }
  
  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if(measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
    measureToSend2 = measure2.RangeMilliMeter;
     
  } else {
    Serial.print(F("Out of range"));
    measureToSend2 = OUT_OF_RANGE;
  }
   Serial.print(F(" "));
   
  // print sensor three reading
  Serial.print(F("3: "));
  if(measure3.RangeStatus != 4) {
    Serial.print(measure3.RangeMilliMeter);
    measureToSend3 = measure3.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
    measureToSend3 = OUT_OF_RANGE;
  }
   Serial.print(F(" "));
   */
  // print sensor four reading
  Serial.print(F("4: "));
  if(measure4.RangeStatus != 4) {
    Serial.print(measure4.RangeMilliMeter);
    measureToSend4 = measure4.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
    measureToSend4 = OUT_OF_RANGE;
  }
   Serial.print(F(" "));
   
  // print sensor five reading
  Serial.print(F("5: "));
  if(measure5.RangeStatus != 4) {
    Serial.print(measure5.RangeMilliMeter);
    measureToSend5 = measure5.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
    measureToSend5 = OUT_OF_RANGE;
  }
   Serial.print(F(" "));
   
  // print sensor six reading
  Serial.print(F("6: "));
  if(measure6.RangeStatus != 4) {
    Serial.print(measure6.RangeMilliMeter);
    measureToSend6 = measure6.RangeMilliMeter;
  } else {
    Serial.print(F("Out of range"));
    measureToSend6 = OUT_OF_RANGE;
  }
   Serial.print(F(" "));
  
  Serial.println();
}




/*
 * função que configura a I2C com os pinos desejados 
 * sda_pin -> 22
 * scl_pin -> 23
 */
void initI2C() 
{
  //Serial.println("---inside initI2C");
  Wire.begin(sda1_pin, scl1_pin);
  bool success = WireSlave1.begin(sda2_pin, scl2_pin, address);
  if(!success){
    Serial.println("Falha na inicializacao I2C escravo");
    while(1){
      delay(100);
    }
  }
  WireSlave1.onReceive(receiveEvent);
  WireSlave1.onRequest(requestEvent);
}

/*
 * função que escreve um dado valor em um dado registro
 */
void writeRegMPU(int reg, int val)      //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                      // escreve o valor no registro
  Wire.endTransmission(true);           // termina a transmissão
}
 
/*
 * função que lê de um dado registro
 */
uint8_t readRegMPU(uint8_t reg)        // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                   // lê o byte e guarda em 'data'
  return data;                          //retorna 'data'
}
 
/*
 * função que procura pelo sensor no endereço 0x68
 */
void findMPU(int mpu_addr)
{
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);
 
  if(data == 0)
  {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
  }
  else
  {
    Serial.println("Dispositivo não encontrado!");
  }
}
 
/*
 * função que verifica se o sensor responde e se está ativo
 */
void checkMPU(int mpu_addr)
{
  findMPU(MPU_ADDR);
     
  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
   
  if(data == 104) 
  {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");
 
    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B
 
    if(data == 64) Serial.println("MPU6050 em modo SLEEP! (64)");
    else Serial.println("MPU6050 em modo ACTIVE!"); 
  }
  else Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
}
 
/*
 * função de inicialização do sensor
 */
void initMPU()
{
  setSleepOff();
  setGyroScale();
  setAccelScale();
}
 
/* 
 *  função para configurar o sleep bit  
 */
void setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}
 
/* função para configurar as escalas do giroscópio
   registro da escala do giroscópio: 0x1B[4:3]
   0 é 250°/s
 
    FS_SEL  Full Scale Range
      0        ± 250 °/s      0b00000000
      1        ± 500 °/s      0b00001000
      2        ± 1000 °/s     0b00010000
      3        ± 2000 °/s     0b00011000
*/
void setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0);
}
 
/* função para configurar as escalas do acelerômetro
   registro da escala do acelerômetro: 0x1C[4:3]
   0 é 250°/s
 
    AFS_SEL   Full Scale Range
      0           ± 2g            0b00000000
      1           ± 4g            0b00001000
      2           ± 8g            0b00010000
      3           ± 16g           0b00011000
*/
void setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0);
}
 
/* função que lê os dados 'crus'(raw data) do sensor
   são 14 bytes no total sendo eles 2 bytes para cada eixo e 2 bytes para temperatura:
 
  0x3B 59 ACCEL_XOUT[15:8]
  0x3C 60 ACCEL_XOUT[7:0]
  0x3D 61 ACCEL_YOUT[15:8]
  0x3E 62 ACCEL_YOUT[7:0]
  0x3F 63 ACCEL_ZOUT[15:8]
  0x40 64 ACCEL_ZOUT[7:0]
 
  0x41 65 TEMP_OUT[15:8]
  0x42 66 TEMP_OUT[7:0]
 
  0x43 67 GYRO_XOUT[15:8]
  0x44 68 GYRO_XOUT[7:0]
  0x45 69 GYRO_YOUT[15:8]
  0x46 70 GYRO_YOUT[7:0]
  0x47 71 GYRO_ZOUT[15:8]
  0x48 72 GYRO_ZOUT[7:0]
    
*/
void readRawMPU()
{  
  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);                       // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)
 
  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo
  AcX |= Wire.read();                     // depois lê o bit menos significativo
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();
 
  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();
 
  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  GyZ = Wire.read() << 8;
  GyZ |= Wire.read(); 
 
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY); 
  Serial.print(" | AcZ = "); Serial.print(AcZ); 
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); 
  Serial.print(" | GyX = "); Serial.print(GyX); 
  Serial.print(" | GyY = "); Serial.print(GyY); 
  Serial.print(" | GyZ = "); Serial.println(GyZ); 
 
                                         
}


void setup() {
  Serial.begin(115200);

  while (! Serial) { delay(1); }
  
  initI2C();

  Serial.println("Iniciando configuracao sensores de distancia. ");
  
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);
  pinMode(SHT_LOX5, OUTPUT);
  pinMode(SHT_LOX6, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  
  
  Serial.println(F("Sensores de distancia prontos"));
  setID();


  
 
  Serial.println("nIniciando configuração do MPU6050 (Acelerometro)n");
  initMPU();
  checkMPU(MPU_ADDR);
 
 
 
  Serial.println("nConfiguração acelerometro finalizada, iniciando loopn");  
}
 
void loop() {
  //readRawMPU();    // lê os dados do acelerometro
  read_dual_sensors(); // lê dados sensor de distancia
  WireSlave1.update();
  delay(1);  
}

void receiveEvent(int howMany){
  command = WireSlave1.read();
  Serial.print("Command received: ");
  Serial.println(command);
  if(command == 'd'){
    long lastReadMillis = millis();
    read_dual_sensors();
    Serial.print("Time in function: ");
    Serial.println(millis() - lastReadMillis);
    Serial.println();
  }else if(command == 'a'){
    readRawMPU();
    Serial.println();
  }
}


void requestEvent(){
  if(command == 'd'){
     
    WireSlave1.write(measureToSend1 >> 8);
    WireSlave1.write(measureToSend1);
    
    WireSlave1.write(measureToSend2 >> 8);
    WireSlave1.write(measureToSend2);
    
    WireSlave1.write(measureToSend3 >> 8);
    WireSlave1.write(measureToSend3);

    WireSlave1.write(measureToSend4 >> 8);
    WireSlave1.write(measureToSend4);

    WireSlave1.write(measureToSend5 >> 8);
    WireSlave1.write(measureToSend5);

    WireSlave1.write(measureToSend6 >> 8);
    WireSlave1.write(measureToSend6);
    
    return;
    
    
  }else if(command == 'a'){
    WireSlave1.write(AcX >> 8);
    WireSlave1.write(AcX);

    WireSlave1.write(AcY >> 8);
    WireSlave1.write(AcY);

    WireSlave1.write(AcZ >> 8);
    WireSlave1.write(AcZ);

    WireSlave1.write(Tmp >> 8);// temperatura == Tmp/340.00+36.56
    WireSlave1.write(Tmp);

    WireSlave1.write(GyX >> 8);
    WireSlave1.write(GyX);

    WireSlave1.write(GyY >> 8);
    WireSlave1.write(GyY);

    WireSlave1.write(GyZ >> 8);
    WireSlave1.write(GyZ);
        
    return;
    
  }else{
   
    WireSlave1.write("error.");
    return;
  }
  
}
