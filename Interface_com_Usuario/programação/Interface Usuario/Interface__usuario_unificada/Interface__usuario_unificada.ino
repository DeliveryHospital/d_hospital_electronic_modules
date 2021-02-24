#include <MFRC522.h>
#include <SPI.h>
#include <Wire.h>
#include <WirePacker.h>
#include <WireSlaveRequest.h>


#define SS_PIN 21
#define RST_PIN 22

#define SDA_COMMAND_PIN 27
#define SCL_COMMAND_PIN 14
#define SLV_ADR 0x15
#define MAX_SLAVE_RESPONSE_LENGTH 32

#define SIZE_BUFFER     18
#define MAX_SIZE_BLOCK  16

#define pinLock     12
#define button     15
int buttonState = 0;


/*esse objeto 'chave' é utilizado para autenticação*/
MFRC522::MIFARE_Key key;
/*código de status de retorno da autenticação*/
MFRC522::StatusCode status;

/* Definicoes pino modulo RC522*/
MFRC522 mfrc522(SS_PIN, RST_PIN);


void setup(){
  /* Inicia a serial*/
  Serial.begin(115200);
  Wire.begin(SDA_COMMAND_PIN, SCL_COMMAND_PIN);
  
  /* Inicia  SPI bus */
  SPI.begin();

  pinMode(pinLock, OUTPUT);
  pinMode (button, INPUT);
  
  /* Inicia MFRC522 */   
  mfrc522.PCD_Init();
  Serial.println("Aproxime o seu cartao/TAG do leitor");
  Serial.println();  
}

void loop() {
  /* Busca novos cartões */
  if ( ! mfrc522.PICC_IsNewCardPresent())
  {
    return;
  }
  /* Seleciona um catão a ser lido */
  if ( ! mfrc522.PICC_ReadCardSerial())
  {
    return;
  }
  /*imprime os detalhes tecnicos do cartão/tag */
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); 

  String content = "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++){
    content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }

  content.toUpperCase();
  Serial.println(content);
  Serial.println();
  bool isCodeValid = verifyAccessCode(content);
  bool readyToOpen = veryfyDoorStatus();

  if((content == " A3 44 A9 92" || isCodeValid)&&(readyToOpen)){  /* Para o robo, procurar o codigo do cartao em um banco de dados e registrar em um log quem acessou*/
    Serial.println("Acesso Permitido");
    digitalWrite(pinLock, HIGH);
    //implementar abertura das portas
    buttonState = digitalRead(button);
    while (buttonState == LOW ){
      buttonState = digitalRead(button);
      }
    digitalWrite(pinLock, LOW);
  }
  else if(!readyToOpen){
    Serial.println("Acesso negado: carro em movimento");
  }else if(!(content == " A3 44 A9 92" || isCodeValid)){
    Serial.println("Acesso negado: cartao invalido");
  }else{
    Serial.println("Acesso negado por motivo desconhecido");
  }
  
 
  /* instrui o PICC quando no estado ACTIVE a ir para um estado de "parada"*/
  mfrc522.PICC_HaltA(); 
  /* "stop" a encriptação do PCD, deve ser chamado após a comunicação com autenticação, caso contrário novas comunicações não poderão ser iniciadas*/
  mfrc522.PCD_StopCrypto1();  
}

bool verifyAccessCode(String code){
  bool canAccess;
  char toSend[30];
  char typeOfRequest = 'c'; // code, not door
  code.toCharArray(toSend, 30);
  WirePacker packer;
  packer.write(typeOfRequest);
  packer.write(toSend);
  packer.end();
  Wire.beginTransmission(SLV_ADR);
  while(packer.available()){
    Wire.write(packer.read());
  }
  Wire.endTransmission();
  //delay(50); /*Delay if function to verify code takes long to run*/
  WireSlaveRequest slaveReq(Wire, SLV_ADR, MAX_SLAVE_RESPONSE_LENGTH);
  bool success = slaveReq.request();
  if(success){
    canAccess = slaveReq.read();
  }else{
    Serial.println("Failed to request validation through I2C");
  }
  return canAccess;
}

bool verifyDoorStatus(){
  bool canOpen;
  char typeOfRequest = 'd'; //door, not code
  WirePacker packer;
  packer.write(typeOfRequest);
  packer.end;
  Wire.beginTransmission(SLV_ADR);
  while(packer.available()){
    Wire.write(packer.read());
  }
  Wire.endTransmission();
  WireSlaveRequest slaveReq(Wire, SLV_ADR, MAX_SLAVE_RESPONSE_LENGTH);
  bool success = slaveReq.request();
  if(success){
    canOpen = slaveReq.read();
  }else{
    Serial.println("Failed to request status through I2C");
  }
  return canOpen;
}

void leituraDados()/*Alem da chave do cartao*/
{
  /*imprime os detalhes tecnicos do cartão/tag */
  mfrc522.PICC_DumpDetailsToSerial(&(mfrc522.uid)); 
  

  /*Prepara a chave - todas as chaves estão configuradas para FFFFFFFFFFFFh (Padrão de fábrica).*/
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  /*buffer para colocar os dados ligos*/
  byte buffer[SIZE_BUFFER] = {0};

  /*bloco que faremos a operação*/
  byte bloco = 1;
  byte tamanho = SIZE_BUFFER;


  /*faz a autenticação do bloco que vamos operar*/
  status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, bloco, &key, &(mfrc522.uid)); /*line 834 of MFRC522.cpp file*/
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Authentication failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    
    return;
  }


  /*faz a leitura dos dados do bloco*/
  status = mfrc522.MIFARE_Read(bloco, buffer, &tamanho);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
   
    return;
  }
  else{
      
  }

  Serial.print(F("\nDados bloco ["));
  Serial.print(bloco);Serial.print(F("]: "));

  /*imprime os dados lidos*/
  for (uint8_t i = 0; i < MAX_SIZE_BLOCK; i++)
  {
      Serial.write(buffer[i]);
  }
  Serial.println(" ");
}
