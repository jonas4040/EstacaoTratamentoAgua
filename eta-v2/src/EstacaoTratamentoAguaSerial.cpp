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
#include "freertos/timers.h"

//-------VARIAVEIS E CONSTANTES COMUNS---------
#define DEBUG_MODE_ON 0
#define TURB1 34
//#define TURB2 35
#define ph 33
#define BOMBA 26
#define MISTURADOR 25
#define BOIA 27

#define TIME_BOIL 0.033333333 * 5.0 /* (!) 2s X 5 = 10s */
// #define TIME_BOIL 10              /* (!) 10min X 60s = 600s = 600000ms */


int aph = 0;
float ackph = 0.0;
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

QueueHandle_t filaTurbidezHandle;
SemaphoreHandle_t serialSemaphore;

TimerHandle_t timerBoilHandle;

UBaseType_t memoriaLivre;
UBaseType_t espacosQueue;
/*
  Prototipos de funçoes comuns
*/

float calcPH();
float calcNTU(float);
void exibeMemoriaDisponivel(TaskHandle_t);
void exibeEspacosQueue(QueueHandle_t);
void escreverSerial(const char *);
void escreverSerial(float);
/*
  Prototipos freertos
*/
void vBombaDaguaTask(void *pvParams);
void vMisturadorTask(void *pvParams);
void vBoiaTask(void *pvParams);
void vTurbidezTask(void *pvParams);
void timerBoil(TimerHandle_t);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  /* Tarefas de comunicacao = PROtocol*/
  serialSemaphore = xSemaphoreCreateMutex();
  xSemaphoreGive(serialSemaphore);

  filaTurbidezHandle = xQueueCreate(1,sizeof(float));

  timerBoilHandle = xTimerCreate("TIMER_BOIA",pdMS_TO_TICKS(TIME_BOIL*60*1000),pdFALSE,0,timerBoil);

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
    nivelBoil = digitalRead(BOIA);
	vTaskDelay(pdMS_TO_TICKS(10));
    if(DEBUG_MODE_ON){
      escreverSerial("Boia: ");
      escreverSerial(nivelBoil?"VERDADEIRO\n":"FALSO\n");
    }
    /* queue dos sensores de turbidez*/
    xQueueReceive(filaTurbidezHandle,(void *)&valorTbd,portMAX_DELAY);
    exibeEspacosQueue(filaTurbidezHandle);

    if(nivelBoil  || valorTbd >= 130){
      //Se precisar usar a boia
      escreverSerial("Já encheu, desligando a bomba (alguns segundos) . . .\n");
      digitalWrite(BOMBA,LOW);  
    }else if(!nivelBoil || valorTbd < 130){
      escreverSerial("Já estamos ligando a bomba novamente (isso pode demorar um pouco). . .\n");
      if(DEBUG_MODE_ON)
        escreverSerial("Iniciando timer da boia . . .\n");
      if(xTimerIsTimerActive(timerBoilHandle)==pdFALSE)
        xTimerStart(timerBoilHandle,0);
    }
    
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

void timerBoil(TimerHandle_t timerHandle){
  /* Callback do timer para manter a boia desligada uns minutos antes de desligar*/
  digitalWrite(BOMBA,HIGH);  
}

/*
    Funç~oes comuns
*/

float calcNTU(float voltagem){
    float valorNTU;
    valorNTU= map(voltagem, 0, 4095, 1000, 0);
    // -(1120.4*voltagem*voltagem)+(5742.3*voltagem)-4352.9;
    return valorNTU;
}

float calcPH(){
  float milivolt;
  milivolt= analogRead(ph);
  ackph=map(milivolt,0,4095,0.00,14.00);
  
  return ackph;
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

/*
  PARA DEBUG
*/
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