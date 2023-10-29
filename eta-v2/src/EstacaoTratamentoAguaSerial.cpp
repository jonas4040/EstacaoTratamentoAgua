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
#include "freertos/queue.h"
#include "freertos/semphr.h"

//-------VARIAVEIS E CONSTANTES COMUNS---------
#define DEBUG_MODE_ON 0
#define TURB1 34
//#define TURB2 35
#define ph 33
#define BOMBA 26
#define MISTURADOR 25
#define BOIA 27
int aturb1 = 0;
int aturb2 = 0;
int aph = 0;
int atemp = 0;
float aux1 = 0.0;
float auxTurb1 = 0.0;
float ackturb2 = 0.0;
float ackph = 0.0;
float acktemp = 0.0;
float sensorValue = 0.0;
//LiquidCrystal_I2C //lcd(0x27,20,4);//VCC no Vin SDA 21 SCL 22

/*
  Variaveis tasks FreeRTOS
*/
TaskHandle_t tbd1Handle = NULL;
TaskHandle_t tbd2Handle = NULL;
TaskHandle_t boiaHandle = NULL;
TaskHandle_t bombaDaguaHandle = NULL;
TaskHandle_t misturadorHandle = NULL;
UBaseType_t memoriaLivre;
UBaseType_t espacosQueue;
QueueHandle_t filaBoiaHandle;
QueueHandle_t filaTurbidezHandle;
SemaphoreHandle_t serialSemaphore;

float calcPH();
float calcNTU(float);
float leSensorTbd(int);
void nivelBoiaISR();
void ligaMisturador(uint8_t,float);

/*
  Prototipos freertos
*/
void vBombaDaguaTask(void *pvParams);
void vMisturadorTask(void *pvParams);
void vBoiaTask(void *pvParams);
void vTurbidezTask(void *pvParams);
void exibeMemoriaDisponivel(TaskHandle_t);
void exibeEspacosQueue(QueueHandle_t);
void escreverSerial(const char *);
void escreverSerial(float);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  /* Tarefas de comunicacao = PROtocol*/
  serialSemaphore = xSemaphoreCreateMutex();
 	xSemaphoreGive(serialSemaphore);
  filaBoiaHandle= xQueueCreate(1,sizeof(bool));
  filaTurbidezHandle = xQueueCreate(1,sizeof(float));

  xTaskCreatePinnedToCore(vBombaDaguaTask,"BOMBA_DAGUA",configMINIMAL_STACK_SIZE+2048,NULL,1,&bombaDaguaHandle,PRO_CPU_NUM);
  xTaskCreatePinnedToCore(vTurbidezTask,"TURBIDEZ_1",configMINIMAL_STACK_SIZE+2048,(void *)TURB1,1,&tbd1Handle,PRO_CPU_NUM);
  // xTaskCreatePinnedToCore(vTurbidezTask,"TURBIDEZ_2",configMINIMAL_STACK_SIZE+2048,(void *)TURB2,3,&tbd2Handle,PRO_CPU_NUM);


  /* Tarefas comuns = APPlication*/
  xTaskCreatePinnedToCore(vMisturadorTask,"MISTURADOR",configMINIMAL_STACK_SIZE+1024,(void *)18,2,&misturadorHandle,APP_CPU_NUM);


  pinMode(TURB1, INPUT);
  // pinMode(TURB2, INPUT);
  // pinMode(ph, INPUT);
  pinMode(BOIA,INPUT);

//  pinMode(phTemp, INPUT);
  attachInterrupt(digitalPinToInterrupt(BOIA),nivelBoiaISR,CHANGE);
  
  pinMode(BOMBA, OUTPUT);
  escreverSerial("Ligando a bomba (Em alguns segundos). . .\n");
  digitalWrite(BOMBA,HIGH); 
  
}

void loop() {
  vTaskDelay(1000);
}

/*
    TASKS
*/
void vBombaDaguaTask(void *pvParams){
  bool nivelBoil;
  float valorTbd;
  while (1){
    //nivelBoil = false;
    
    /* queue da boia*/
    xQueueReceive(filaBoiaHandle,(void *)&nivelBoil,portMAX_DELAY);
    
    /* queue dos sensores de turbidez*/
    // if(xQueueReceive(filaTurbidezHandle,&valorTbd,portMAX_DELAY) != pdTRUE){
    //   xQueueReset(filaBoiaHandle);
    // }
    xQueueReceive(filaTurbidezHandle,(void *)&valorTbd,portMAX_DELAY);
    exibeEspacosQueue(filaTurbidezHandle);

    
    escreverSerial(valorTbd);
    escreverSerial(" <--- \n");

    if(nivelBoil  || valorTbd >= 130){
      //Se precisar usar a boia
      escreverSerial("Já encheu, desligando a bomba (alguns segundos) . . .\n");
      digitalWrite(BOMBA,LOW);  
    }else if(!nivelBoil || valorTbd < 130){
      escreverSerial("Já estamos ligando a bomba novamente (alguns segundos). . .\n");
      vTaskDelay(pdMS_TO_TICKS(2000));
      digitalWrite(BOMBA,HIGH);  
    }
    escreverSerial("Boia: ");
    escreverSerial(nivelBoil?"Vai transbordar\n":"Enchendo\n");
    exibeMemoriaDisponivel(NULL);

  }
  
}

void vMisturadorTask(void *pvParams){
    int segundos = (int)pvParams;
    pinMode(MISTURADOR, OUTPUT);
    
    while (1){
      digitalWrite(MISTURADOR,HIGH);
      vTaskDelay(pdMS_TO_TICKS(1000*segundos));
      digitalWrite(MISTURADOR,LOW);
      vTaskDelay(pdMS_TO_TICKS(1000*segundos));
      exibeMemoriaDisponivel(NULL);
    }
    
}

void vTurbidezTask(void *pvParams){
  uint8_t pinoTurb = (int) pvParams;
  float NTU = 0.0;
  float voltage = 0.0;

  while(1){
    sensorValue = analogRead(pinoTurb);
    sensorValue+=1494; 
    voltage = sensorValue * (3.2 / 4095);  //anes esava 3.2v

    NTU = calcNTU(sensorValue);
    xQueueOverwrite(filaTurbidezHandle,(void *)&NTU);
    // if(xQueueSend(filaTurbidezHandle,&NTU,portMAX_DELAY) == pdTRUE){
    //   escreverSerial("Enviou -->");
    // }else{
    //   break;
    // }
    vTaskDelay(pdMS_TO_TICKS(200));


    //escreverSerial(voltage);
    escreverSerial("Turbidez ");
    escreverSerial(pinoTurb == TURB1 ? "1 " : "2 ");
    escreverSerial("-> ");
    escreverSerial(NTU);
    escreverSerial(" NTU\n");
    
    exibeMemoriaDisponivel(NULL);
  }
}

void escreverSerial(float number) {
	if (xSemaphoreTake(serialSemaphore, portMAX_DELAY) == pdTRUE) {
		Serial.print(number);
		Serial.print(" ");
		xSemaphoreGive(serialSemaphore);
	}
}

void escreverSerial(const char *str) {
	if (xSemaphoreTake(serialSemaphore, portMAX_DELAY) == pdTRUE) {
		Serial.print(str);
		xSemaphoreGive(serialSemaphore);
	}
}

/* PARA DEBUG */
void exibeMemoriaDisponivel(TaskHandle_t handleTask){
  if(DEBUG_MODE_ON){
    memoriaLivre = uxTaskGetStackHighWaterMark(handleTask);
    escreverSerial("Memoria Livre Task");
    escreverSerial(pcTaskGetName(handleTask));
    escreverSerial(" : ");
    escreverSerial(memoriaLivre);
    escreverSerial("B \n");
  }
  
}

void exibeEspacosQueue(QueueHandle_t xQueue){
  if(DEBUG_MODE_ON){
    espacosQueue = uxQueueSpacesAvailable(xQueue);
    escreverSerial("Espaços Disponiveis nas Filas (turbidez): ");
    escreverSerial(espacosQueue);
    escreverSerial(" \n");
  }
}



float calcNTU(float voltagem){
    float valorNTU;
    valorNTU= map(voltagem, 0, 4095, 1000, 0);
    // -(1120.4*voltagem*voltagem)+(5742.3*voltagem)-4352.9;
    return valorNTU;
}

void nivelBoiaISR(){
  //Lógica da boia
  bool nivelBoil = digitalRead(BOIA);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;


  xQueueOverwriteFromISR(filaBoiaHandle,(void *)&nivelBoil,&xHigherPriorityTaskWoken);


  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void ligaMisturador(uint8_t pino, float minutos){
    digitalWrite(pino,HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000*60*minutos));
    digitalWrite(pino,LOW);
    vTaskDelay(pdMS_TO_TICKS(1000*60*minutos));
}

float calcPH(){
  float milivolt;
  milivolt= analogRead(ph);
  ackph=map(milivolt,0,4095,0.00,14.00);
  
  return ackph;
}

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