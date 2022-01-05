#include <Wire.h>
#include <WireSlave.h>
#include <WirePacker.h>

#define SDA_PIN_READER 21
#define SCL_PIN_READER 22
#define SLV_ADR 0x15

#define SDA_PIN_MAIN 27
#define SCL_PIN_MAIN 14

char command;
bool isCodeValid = false;
bool isDoorAvailable = false;


void setup() {
  Wire.begin(SDA_PIN_MAIN, SCL_PIN_MAIN);
  bool success = WireSlave1.begin(SDA_PIN_READER, SCL_PIN_READER, SLV_ADR);
  if(!success){
    Serial.println("Falha na inicializacao do I2C com o leitor");
    while(1){
      delay(100);
    }
  }
  WireSlave1.onReceive(receiveEvent);
  WireSlave1.onRequest(requestEvent);
  Serial.begin(115200);
}

void loop() {
  WireSlave1.update();
  delay(1);
  // put your main code here, to run repeatedly:

}

void receiveEvent(int howMany){
  command = WireSlave1.read();
  Serial.print("Command received: ");
  Serial.println(command);
  if(command == 'c'){
    char code[30];
    for(int i = 0; i < 30; i++){
      code[i] = WireSlave1.read();
    }
    isCodeValid = verifyCode(code);
    
  }else if(command == 'd'){
    isDoorAvailable = verifyDoorStatus();

  }
}


void requestEvent(){
  if(command == 'c'){
    WireSlave1.write(isCodeValid);
  }else if(command == 'd'){
    WireSlave1.write(isDoorAvailable);
  }else{
    WireSlave1.write(false);
  }
}

bool verifyCode(char code[]){
  //implementar a funcao usnado algum banco de dados dos codigos dos crachas
  //aqui tambem registrar em algum log quem acessou e em que horario

  return true;
}

bool verifyDoorStatus(){
  //perguntar pra o driver se o robo esta em movimento, caso contrario porta n abre

  return true;
}
