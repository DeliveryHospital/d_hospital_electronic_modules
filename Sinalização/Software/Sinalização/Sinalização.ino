#include <FastLED.h>


/*-------------------------------------------------------------------------*/
/*--------------------------VARIAVEIS ALTERAVEIS---------------------------*/
/*-------------------------------------------------------------------------*/
//pinos para emissão de sinal para virar para a esquerda ou para direita
#define ESQUERDA_PIN 12
#define DIREITA_PIN 13
#define NUM_FAROIS 2
#define FAROL_TYPE NEOPIXEL

//pino do farol da camera 
#define CAMERA_PIN 14
#define NUM_CAMERA 1

//pino do buzzer
#define BUZZER_PIN 16
#define BUZZER_FREQUENCY 1000

/*-----LEDS em geral----------*/
//pinos de saida de sinal dos leds 
#define LED_PIN1 4
#define LED_PIN2 2
#define LED_PIN3 15
//numeros de leds por pinos
#define NUM_LEDS1 80
#define NUM_LEDS2 295
#define NUM_LEDS3 80
//brilho de cada sinal
#define BRIGHTNESS 255
#define BRIGHTNESS1 255
#define BRIGHTNESS2 255
#define BRIGHTNESS3 255
//taxa de atualização de cada led
#define UPDATES_PER_SECOND 80
#define UPDATES_PER_SECOND1 80   //ATUALIZACAO DO DISPLAY DE LEDs EM 1 SEGUNDO.
#define UPDATES_PER_SECOND2 80   //ATUALIZACAO DO DISPLAY DE LEDs EM 1 SEGUNDO.
#define UPDATES_PER_SECOND3 80   //ATUALIZACAO DO DISPLAY DE LEDs EM 1 SEGUNDO.
//modelo do leds
#define LED_TYPE1 WS2812B
#define LED_TYPE2 WS2812B
#define LED_TYPE3 WS2812B
//orfem das cores
#define COLOR_ORDER1 GRB   // A ORDEM DAS CORES DA TIRA DE LEDs.
#define COLOR_ORDER2 GRB
#define COLOR_ORDER3 GRB
/*----------------------------------------------------------------------------*/
//Declarações 
CRGB leds1[NUM_LEDS1];
CRGB leds2[NUM_LEDS2];
CRGB leds3[NUM_LEDS3];
CRGB esq[NUM_FAROIS];
CRGB dir[NUM_FAROIS];
CRGB cam[NUM_CAMERA];
 

 //palheta de cores para efeitos com os leds
CRGBPalette16 currentPalette;
TBlendType    currentBlending;
 
void setup() {
    //Atraso de segurança na inicialização
    delay(3000);
    
    pinMode(BUZZER_PIN,   OUTPUT);

    //inicialização do Buzzer
    ledcAttachPin(BUZZER_PIN, 0); //Atribuimos o pino BUZZER_PIN ao canal 0.
    ledcSetup(0, BUZZER_FREQUENCY, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
    
    //inicialização dos leds
    FastLED.addLeds<LED_TYPE1, LED_PIN1, COLOR_ORDER1>(leds1, NUM_LEDS1).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE2, LED_PIN2, COLOR_ORDER2>(leds2, NUM_LEDS2).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<LED_TYPE3, LED_PIN3, COLOR_ORDER3>(leds3, NUM_LEDS3).setCorrection(TypicalLEDStrip);
    FastLED.addLeds<FAROL_TYPE, ESQUERDA_PIN>(esq, NUM_FAROIS);
    FastLED.addLeds<FAROL_TYPE, DIREITA_PIN >(dir, NUM_FAROIS);
    FastLED.addLeds<FAROL_TYPE, CAMERA_PIN  >(cam, NUM_CAMERA);
    
    
    FastLED.setBrightness( BRIGHTNESS );
    for(int i=0; i< NUM_LEDS1; i++){ leds1[i] = CRGB::Black;} FastLED.show();
    for(int i=0; i< NUM_LEDS2; i++){ leds2[i] = CRGB::Black;} FastLED.show(); 
    for(int i=0; i< NUM_LEDS3; i++){ leds3[i] = CRGB::Black;} FastLED.show();
    for(int i=0; i< NUM_FAROIS; i++){ esq[i]  = CRGB::Black;} FastLED.show();
    for(int i=0; i< NUM_FAROIS; i++){ dir[i]  = CRGB::Black;} FastLED.show();
    for(int i=0; i< NUM_CAMERA; i++){ cam[i]  = CRGB::Black;} FastLED.show();
    
    currentPalette = RainbowColors_p;   //PALHETA DE CORES SENDO UTILIZADA ATUALMENTE!
    //currentPallete = RainbowStripeColors_p;   //OUTRA PALHETA DE CORES QUE PODE SER UTILIZADA.
    //currentPalette = RainbowColors_p;   //OUTRA PALHETA DE CORES QUE PODE SER UTILIZADA.
    currentBlending = LINEARBLEND;
    Serial.begin(9600);
}
 
void OpenDoor(CRGB leds[],int cor1, int cor2, int cor3){
  int numleds=4;
  if(leds == leds1){numleds= sizeof(leds1)/sizeof(leds1[0]);}
  if(leds == leds2){numleds= sizeof(leds2)/sizeof(leds2[0]);}
  if(leds == leds3){numleds= sizeof(leds3)/sizeof(leds3[0]);}
  int i=( numleds)/2;  int j=numleds/2;
  leds[i] = CRGB(cor1, cor2, cor3);
  FastLED.show();
  FastLED.delay(2000 / UPDATES_PER_SECOND);
  while(i<numleds | j>0){
    i++;
    j--;
    if(i < numleds){
      leds[i] = CRGB(cor1, cor2, cor3);
      }
    if(j >= 0){
      leds[j] = CRGB(cor1, cor2, cor3);
      }
    FastLED.show();
    FastLED.delay(20000 / UPDATES_PER_SECOND);
    }
  }
  
void Trail(CRGB leds[],int cor1, int cor2, int cor3){
  int numleds=4;
  if(leds == leds1){numleds= sizeof(leds1)/sizeof(leds1[0]);}
  if(leds == leds2){numleds= sizeof(leds2)/sizeof(leds2[0]);}
  if(leds == leds3){numleds= sizeof(leds3)/sizeof(leds3[0]);}
  
  for(int i=0; i< numleds; i++){
    leds[i] = CRGB(cor1, cor2, cor3);
    FastLED.show();
    FastLED.delay(2000 / UPDATES_PER_SECOND);
    }
    for(int i=0; i< numleds; i++){
    leds[i] = CRGB::Black;
    FastLED.show();
    FastLED.delay(2000 / UPDATES_PER_SECOND);
    }
  }
  
void TurnOnLeds(CRGB leds[],int cor1, int cor2, int cor3){
  int numleds=4;
  if(leds == leds1){numleds= sizeof(leds1)/sizeof(leds1[0]);}
  if(leds == leds2){numleds= sizeof(leds2)/sizeof(leds2[0]);}
  if(leds == leds3){numleds= sizeof(leds3)/sizeof(leds3[0]);}
   for(int i=0; i< numleds; i++){
    leds[i] = CRGB(cor1, cor2, cor3);
    }
    FastLED.show();
  }

  
void Bounce(uint8_t colorIndex, CRGB leds[]){
    int numleds=4;
    if(leds == leds1){numleds= sizeof(leds1)/sizeof(leds1[0]);}
    if(leds == leds2){numleds= sizeof(leds2)/sizeof(leds2[0]);}
    if(leds == leds3){numleds= sizeof(leds3)/sizeof(leds3[0]);}
  
    uint8_t brightness = 200;
    
    for(int i=0; i< numleds; i++){
    leds[i]= ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
    FastLED.show();
    FastLED.delay(2000 / UPDATES_PER_SECOND1);
    leds[i]= ColorFromPalette( currentPalette, colorIndex, 0, currentBlending);
    FastLED.show();
    FastLED.delay(2000 / UPDATES_PER_SECOND1);
    colorIndex += 3;
    }
  }

void Clear(uint8_t colorIndex, CRGB leds[]){
    int numleds=4;
    if(leds == leds1){numleds= sizeof(leds1)/sizeof(leds1[0]);}
    if(leds == leds2){numleds= sizeof(leds2)/sizeof(leds2[0]);}
    if(leds == leds3){numleds= sizeof(leds3)/sizeof(leds3[0]);}
    
  for(int i=0; i< numleds; i++){
    leds[i] = CRGB::Black;
    FastLED.show();
    colorIndex += 3;
    }
 }
  
void FillLEDsFromPaletteColors(uint8_t colorIndex, CRGB leds[]){
    int numleds=4;
    if(leds == leds1){numleds= sizeof(leds1)/sizeof(leds1[0]);}
    if(leds == leds2){numleds= sizeof(leds2)/sizeof(leds2[0]);}
    if(leds == leds3){numleds= sizeof(leds3)/sizeof(leds3[0]);}
    
    //uint8_t brightness = 13;   //AUMENTA OU DIMINUI A "INTENSIDADE DE LUMINOSIDADE" QUE OS LEDs EMITEM (1 ~ 255) R.COSTA.
    uint8_t brightness = 200;   //AUMENTA OU DIMINUI A "INTENSIDADE DE LUMINOSIDADE" QUE OS LEDs EMITEM (1 ~ 255) R.COSTA.
    
    for( int i = 0; i < numleds; i++){
        leds[i] = ColorFromPalette( OceanColors_p, colorIndex, brightness, currentBlending);
        ///leds1[i]= ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        ///leds1[i]= ColorFromPalette( currentPalette, colorIndex, 0, currentBlending);
        colorIndex += 3;
    }
}

void FillLEDsFromPaletteColors2(uint8_t colorIndex, CRGB leds[]){
    int numleds=4;
    if(leds == leds1){numleds= sizeof(leds1)/sizeof(leds1[0]);}
    if(leds == leds2){numleds= sizeof(leds2)/sizeof(leds2[0]);}
    if(leds == leds3){numleds= sizeof(leds3)/sizeof(leds3[0]);}
    
    //uint8_t brightness = 13;   //AUMENTA OU DIMINUI A "INTENSIDADE DE LUMINOSIDADE" QUE OS LEDs EMITEM (1 ~ 255) R.COSTA.
    uint8_t brightness = 200;   //AUMENTA OU DIMINUI A "INTENSIDADE DE LUMINOSIDADE" QUE OS LEDs EMITEM (1 ~ 255) R.COSTA.
    
    for( int i = 0; i < numleds; i++){
        leds[i] = ColorFromPalette( LavaColors_p, colorIndex, brightness, currentBlending);
        ///leds1[i]= ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        ///leds1[i]= ColorFromPalette( currentPalette, colorIndex, 0, currentBlending);
        colorIndex += 3;
    }
}
 
// Existem várias palhetas de efeitos luminosos e diferentes cores demonstradas aqui.
// a biblioteca FastLED varias palhetas já configuradas como opcao de uso, como por exemplo:
// RaibowColors_p, RainbowColors_p, OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p and PartyColors_p
// Além disso, você pode digitar ou definir manualmente suas próprias palhetas de cores.
 
void ChangePalettePeriodically(){
    uint8_t secondHand = (millis() / 1000) % 60;
    static uint8_t lastSecond = 99;
    if(lastSecond != secondHand){
        lastSecond = secondHand;
        if(secondHand ==  0){ currentPalette = RainbowColors_p; currentBlending = LINEARBLEND;}
    }
}

void BuzzerBeepOn(int DutyCycle){
   ledcWrite(0, DutyCycle);//Escrevemos no canal 0, o duty cycle
   delay(2);
  }

void BuzzerBeepOff(){
   ledcWrite(0,0);//Escrevemos no canal 0, o duty cycle
   delay(2);
  }

void TurnLeft(){
  //while virando:
  for (int i = 0; i < 1; i++){
      esq[i] = CRGB (100, 5, 150);
  }
  FastLED.show();
  delay (500);
  for (int i = 0; i < 1; i++) {
    esq[i] = CRGB (0, 0, 0);
  }
  FastLED.show();
  delay (300);
  }

void TurnRight(){
  //while virando:
  for (int i = 0; i < 1; i++){
      dir[i] = CRGB (100, 5, 150);
  }
  FastLED.show();
  delay (500);
  for (int i = 0; i < 1; i++) {
    dir[i] = CRGB (0, 0, 0);
  }
  FastLED.show();
  delay (300);
  }

  void loop(){
    ChangePalettePeriodically();
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1;
    
        //Trail(leds1,188,143,143);
        /*Trail(leds2,188,143,143);
        Trail(leds3,188,143,143);
        TurnOnLeds(leds1,255,140,0);
        TurnOnLeds(leds2,255,140,0);
        TurnOnLeds(leds3,255,140,0);
        delay(2000);
        Clear(startIndex, leds1);
        Clear(startIndex, leds2);
        Clear(startIndex, leds3);
        OpenDoor(leds1,128,0,128);
        OpenDoor(leds2,128,0,128);
        OpenDoor(leds3,128,0,128);
        Clear(startIndex, leds1);
        //Clear(startIndex, leds2);
        //Clear(startIndex, leds3);*/
        TurnLeft();
        TurnLeft();
        TurnLeft();
        TurnLeft();
        TurnRight();
        TurnRight();
        TurnRight();
        TurnRight();
        //BuzzerBeepOn(1023);
        //delay(5000);
        //BuzzerBeepOff(); 
        //FillLEDsFromPaletteColors(startIndex,leds1);
        //FillLEDsFromPaletteColors2(startIndex,leds2);
        
        
        
    FastLED.show();
    FastLED.delay(2000 / UPDATES_PER_SECOND);
   
}


/*---------------------------------------------------------------
---------------------------Tabela de cores-----------------------
----------------------------------------------------------------*/
/*

--Tons de Cinza - Gray Colors--  
Nome da Cor = Código RGB
Black = (0,0,0)
grey11  = (28,28,28)
grey21  = (54,54,54)
grey31  = (79,79,79)
DimGray = (105,105,105)
Gray  = (128,128,128)
DarkGray  = (169,169,169)
Silver  = (192,192,192)
LightGrey = (211,211,211)
Gainsboro = (220,220,220)

--Tons de Azul - Blue Colors--    
SlateBlue  = (106,90,205)
SlateBlue1  = (131,111,255)
SlateBlue3  = (105,89,205)
DarkSlateBlue = (72,61,139)
MidnightBlue  = (25,25,112)
Navy  = (0,0,128)
DarkBlue  = (0,0,139)
MediumBlue  = (0,0,205)
Blue  = (0,0,255)
CornflowerBlue  = (100,149,237)
RoyalBlue = (65,105,225)
DodgerBlue  = (30,144,255)
DeepSkyBlue = (0,191,255)
LightSkyBlue  = (135,206,250)
SkyBlue = (135,206,235)
LightBlue = (173,216,230)
SteelBlue = (70,130,180)
LightSteelBlue  = (176,196,222)
SlateGray = (112,128,144)
LightSlateGray  = (119,136,153)


*/
