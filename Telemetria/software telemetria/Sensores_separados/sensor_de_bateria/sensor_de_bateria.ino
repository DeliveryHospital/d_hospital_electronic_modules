#define BATERIA_PIN 13

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  float bateria = float(analogRead(BATERIA_PIN)*41/4095);
  Serial.print("bateria:");
  Serial.print(bateria);
  Serial.print("[V] \n");
  delay(1000);
}
