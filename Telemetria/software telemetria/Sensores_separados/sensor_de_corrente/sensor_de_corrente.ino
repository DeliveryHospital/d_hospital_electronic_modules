float vetCorrente1[300];
float valor_medio1;
float valor_Corrente1;
int pin_corrente1=14;

float vetCorrente2[300];
float valor_medio2;
float valor_Corrente2;
int pin_corrente2=27;

void setup()
{
  Serial.begin(9600);
  pinMode(pin_corrente1, INPUT);
  pinMode(pin_corrente2, INPUT);
}
 
void loop()
{
  double maior_Valor1 = 0;
  double maior_Valor2 = 0;
  double valor_Corrente1 = 0;
  double valor_Corrente2 = 0;    
 
  float tensao1 = 127;
  float tensao2 = 127;
  float potencia1 = 0;
  float potencia2 = 0;
 
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
  
  Serial.print(".");
  delay(500);
  Serial.print(".");
  delay(500);
  Serial.print(".");
  delay(500);
  Serial.println("");
}
