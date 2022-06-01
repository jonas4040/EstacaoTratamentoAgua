#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//-------VARIAVEIS E CONSTANTES---------
#define turb1 34
#define turb2 35
#define ph 33
#define rele1 25
#define rele2 26
#define boia 27
int aturb1 = 0;
int aturb2 = 0;
int aph = 0;
int atemp = 0;
float aux1 = 0.0;
float auxTurb1 = 0.0;
float  NTU = 0.0;
float voltage;
float ackturb2 = 0.0;
float ackph = 0.0;
float acktemp = 0.0;
float sensorValue = 0.0;


LiquidCrystal_I2C lcd(0x27,20,4);//VCC no Vin SDA 21 SCL 22

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(turb1, INPUT);
  pinMode(turb2, INPUT);
  pinMode(ph, INPUT);
//  pinMode(phTemp, INPUT);
  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  pinMode(boia,INPUT);
  
  lcd.init();
  lcd.backlight();
  Serial.println("Ligando a bomba (Em alguns segundos). . .");
  delay(2000);
  digitalWrite(rele1,HIGH);
}

void loop() {
  //Serial.println("FUNCIONANDO.");
  //testeDeDisplay();
  Serial.print("Tbd 1: ");
  leSensorTbd(turb1);
  Serial.print("Tbd 2: ");
  leSensorTbd(turb2);
  Serial.print("PH: ");
  Serial.println(calcPH());
  delay(300);

  nivelBoia(boia);
}

float calcPH(){
  float milivolt;
  milivolt= analogRead(ph);
  ackph=map(milivolt,0,4095,0.00,14.00);
  
  return ackph;
}

float calcNTU(float voltagem){
    float valorNTU;
    valorNTU= map(voltagem, 0, 4095, 1000, 0);
    // -(1120.4*voltagem*voltagem)+(5742.3*voltagem)-4352.9;
    return valorNTU;
}

float leSensorTbd(int pinoTurb){
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print("ESP 32 ! ! !");
  //escrevaDisplay("ESP 32 ! ! !");
  sensorValue = analogRead(pinoTurb);
  sensorValue+=1494; 
  voltage = sensorValue * (3.2 / 4095);  //anes esava 3.2v

  NTU = calcNTU(sensorValue);

  //O serial interfere no lcd print
  //Serial.println(voltage);  
  Serial.print(NTU);
  Serial.println(" NTU");
  
  //lcd.clear();
 // aux1 = map(sensorValue , 0 , 4095, 1000, 0);
 
 // lcd.setCursor(0,0);
  //lcd.print("Tbd: ");
  //lcd.print(aux1);
  delay(200);
  return NTU;
}

void nivelBoia(int boil){
  //Lógica da boia
  int nivelBoil=digitalRead(boil);
  //Serial.println(nivelBoil);
  if(nivelBoil==HIGH || leSensorTbd(turb1) >=130){
    //Se precisar usar a boia
    Serial.println("Já encheu, desligando a bomba (alguns segundos) . . .");
    Serial.print("Boia: ");
    digitalWrite(rele1,LOW);  
  }else if(nivelBoil == LOW || leSensorTbd(turb1)<130){
    Serial.println("Já estamos ligando a bomba novamente (alguns segundos). . .");
    delay(2000);
    digitalWrite(rele1,HIGH);  
  }
}

void escrevaDisplay(char msg[19]){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(msg);
}

void testeDeDisplay(){
   for(int i=0;i<=7;i++){
    lcd.clear();
    
    lcd.setCursor(2,0);
    lcd.print("pH: ");
    
    lcd.print(i);

    lcd.setCursor(1,1);
    lcd.print("Tbd: ");
    lcd.print(i*10);
    lcd.print(" NTU");
    delay(120);  
  }
}
