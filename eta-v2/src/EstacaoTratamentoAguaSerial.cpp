/**
 * Bibliotecas comuns
*/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/**
 * Bibliotecas FreeRTOS
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//-------VARIAVEIS E CONSTANTES COMUNS---------
#define turb1 34
#define turb2 35
#define ph 33
#define BOMBA 26
#define MISTURADOR 25
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
//LiquidCrystal_I2C //lcd(0x27,20,4);//VCC no Vin SDA 21 SCL 22

/*
  Variaveis tasks FreeRTOS
*/
TaskHandle_t tbdHandle = NULL;
TaskHandle_t boiaHandle = NULL;
TaskHandle_t bombaDaguaHandle = NULL;
TaskHandle_t misturadorHandle = NULL;

float calcPH();
float calcNTU(float);
float leSensorTbd(int);
void nivelBoia(int);
void ligaMisturador(uint8_t,float);

/*
  Prototipos freertos
*/
void vBombaDaguaTask(void *pvParams);
void vMisturadorTask(void *pvParams);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  xTaskCreate(vBombaDaguaTask,"BOMBA_DAGUA",configMINIMAL_STACK_SIZE,NULL,1,&bombaDaguaHandle);
  xTaskCreate(vMisturadorTask,"MISTURADOR",configMINIMAL_STACK_SIZE+1024,(void *)18,2,&misturadorHandle);


  pinMode(turb1, INPUT);
  pinMode(turb2, INPUT);
  pinMode(ph, INPUT);
//  pinMode(phTemp, INPUT);
  pinMode(BOMBA, OUTPUT);
  pinMode(boia,INPUT);
  
  //lcd.init();
  //lcd.backlight();
  Serial.println("Ligando a bomba (Em alguns segundos). . .");
  delay(2000);
  digitalWrite(BOMBA,HIGH);
}

void loop() {
  //testeDeDisplay();
  // Serial.print("Tbd 1: ");
  // leSensorTbd(turb1);
  // Serial.print("Tbd 2: ");
  // leSensorTbd(turb2);
  // Serial.print("PH: ");
  // Serial.println(calcPH());
  // delay(300);

  // nivelBoia(boia);
  vTaskDelay(1000);
}

/*
    TASKS
*/
void vBombaDaguaTask(void *pvParams){
  while (1){
    vTaskDelay(1000);
  }
  
}

void vMisturadorTask(void *pvParams){
    int segundos = (int)pvParams;
    pinMode(MISTURADOR, OUTPUT);
    Serial.println("MISTURADOR");
    
    while (1){
      digitalWrite(MISTURADOR,HIGH);
      vTaskDelay(pdMS_TO_TICKS(1000*segundos));
      digitalWrite(MISTURADOR,LOW);
      vTaskDelay(pdMS_TO_TICKS(1000*segundos));
    }
    
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
  ////lcd.clear();
  ////lcd.setCursor(0,0);
  ////lcd.print("ESP 32 ! ! !");
  //escrevaDisplay("ESP 32 ! ! !");
  sensorValue = analogRead(pinoTurb);
  sensorValue+=1494; 
  voltage = sensorValue * (3.2 / 4095);  //anes esava 3.2v

  NTU = calcNTU(sensorValue);

  //O serial interfere no //lcd print
  //Serial.println(voltage);  
  Serial.print(NTU);
  Serial.println(" NTU");
  
  ////lcd.clear();
 // aux1 = map(sensorValue , 0 , 4095, 1000, 0);
 
 // //lcd.setCursor(0,0);
  ////lcd.print("Tbd: ");
  ////lcd.print(aux1);
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
    digitalWrite(BOMBA,LOW);  
  }else if(nivelBoil == LOW || leSensorTbd(turb1)<130){
    Serial.println("Já estamos ligando a bomba novamente (alguns segundos). . .");
    delay(2000);
    digitalWrite(BOMBA,HIGH);  
  }

  Serial.print("Boia: ");
  Serial.println(nivelBoil);
}

void ligaMisturador(uint8_t pino, float minutos){
    digitalWrite(pino,HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000*60*minutos));
    digitalWrite(pino,LOW);
    vTaskDelay(pdMS_TO_TICKS(1000*60*minutos));
}
// void escrevaDisplay(char msg[19]){
//   //lcd.clear();
//   //lcd.setCursor(0,0);
//   //lcd.print(msg);
// }

// void testeDeDisplay(){
//    for(int i=0;i<=7;i++){
//     //lcd.clear();
    
//     //lcd.setCursor(2,0);
//     //lcd.print("pH: ");
    
//     //lcd.print(i);

//     //lcd.setCursor(1,1);
//     //lcd.print("Tbd: ");
//     //lcd.print(i*10);
//     //lcd.print(" NTU");
//     delay(120);  
//   }
// }

/**
 * TESTES
*/

/*
#include <Arduino.h>
#include <Wire.h>

#define RELE_1 25
#define RELE_2 26
#define BOIA 27
#define TBD_1 34
#define TBD_2 35

void testeReleLiga(uint8_t pino);
void testeLeBoia();
void testeTurbidez(uint8_t pino);
float calcNTU(float voltagem);

void setup(){
    Serial.begin(115200);
    pinMode(RELE_1, OUTPUT);
    pinMode(RELE_2, OUTPUT);
    pinMode(BOIA,INPUT);
}

void loop(){
    // testeReleLiga(RELE_1);
    // testeReleLiga(RELE_2);
    //testeLeBoia();
    testeTurbidez(TBD_1);
}

void testeReleLiga(uint8_t pino){
    uint8_t estado = 0;
    digitalWrite(pino,HIGH);
    estado = !estado;
    delay(3000);
    digitalWrite(pino,LOW);
    delay(3000);
}

void testeLeBoia()
{
  //D27 sensorNivel
  Serial.print("Sensor de n'ivel: ");
  Serial.println(digitalRead(BOIA));
}

void testeTurbidez(uint8_t pino){
  uint16_t sensorValue = analogRead(pino);
  sensorValue+=1494; 
  uint16_t voltage = sensorValue * (3.2 / 4095);  //anes esava 3.2v

  uint16_t NTU = calcNTU(sensorValue);

  //O serial interfere no //lcd print
  //Serial.println(voltage);  
  Serial.print(NTU);
  Serial.println(" NTU");
}

float calcNTU(float voltagem){
    float valorNTU;
    valorNTU= map(voltagem, 0, 4095, 1000, 0);
    // -(1120.4*voltagem*voltagem)+(5742.3*voltagem)-4352.9;
    return valorNTU;
}
*/