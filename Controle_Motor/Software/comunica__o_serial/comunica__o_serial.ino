
int serial_vel = 9600;

int pin_vel = 25;
int pin_rot = 26;

int pin_vel_channel = 0;
int pin_rot_channel = 1;

int frequency = 100000;
int bit_resolution = 10;

unsigned int vel,rot;

void setup() {
  Serial.begin(serial_vel);

  vel = 511;
  rot = 511;
  
  pinMode(pin_vel,OUTPUT);
  pinMode(pin_rot,OUTPUT);

  ledcAttachPin(pin_vel, pin_vel_channel);
  ledcAttachPin(pin_rot, pin_rot_channel);

  ledcSetup(pin_vel_channel, frequency, bit_resolution);
  ledcSetup(pin_rot_channel, frequency, bit_resolution);

  ledcWrite(pin_vel_channel, vel);
  ledcWrite(pin_rot_channel, rot);
}


void loop() {
  //readSerial();
  test_move();
  
 }

void foward(int value){
  ledcWrite(pin_vel_channel, value);
}

void turn(int value){
  ledcWrite(pin_rot_channel, value);
}

void readSerial(){
  unsigned int bytesRead;
    if (Serial.available()) {
        bytesRead = Serial.parseInt();
      if(bytesRead>0){
        if ((bytesRead/(1000*bit_resolution))==0){
          vel = bytesRead%(1000*bit_resolution);
          Serial.print("bytesRead = ");
          Serial.println(bytesRead,HEX);
          Serial.print("vel = ");
          Serial.println(vel,HEX);
          ledcWrite(pin_vel_channel, vel);
          }
         if ((bytesRead/(1000*bit_resolution))==1){
          rot = bytesRead%(1000*bit_resolution);
          Serial.print("bytesRead = ");
          Serial.println(bytesRead,HEX);
          Serial.print("rot = ");
          Serial.println(rot,HEX);
          ledcWrite(pin_rot_channel, rot);
         }
         else{
          ledcWrite(pin_vel_channel, vel);
          ledcWrite(pin_rot_channel, rot);
          }
        
        }
  }
}

void test_foward(){
  foward(511);
  delay(5000);
  foward(1023);
  delay(5000);
  foward(0);
  delay(5000);
}

void test_turn(){
  turn(511);
  delay(5000);
  turn(1023);
  delay(5000);
  turn(0);
  delay(5000);
  }

void test_move(){
  delay(7000);
  turn(511);
  foward(1023);
  delay(5000);
  foward(511);
  turn(1023);
  delay(1500);
  turn(511);
  foward(1023);
  delay(5000);
  foward(511);
  turn(1023);
  delay(1500);
  }
