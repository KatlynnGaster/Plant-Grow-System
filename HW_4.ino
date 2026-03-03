#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include "SparkFunBME280.h"
#include "SparkFunCCS811.h"
#include <ESP32Servo.h>
#include <RTClib.h>

TaskHandle_t GrowLightHandler = NULL;
TaskHandle_t WateringHandler = NULL;
TaskHandle_t EnvironmentHandler = NULL;
TaskHandle_t LoggerHandler = NULL;

#define PHOTO_PIN 36
#define W_LEVEL_PIN 39
#define SERVO_PIN 13
#define WHITE_LED 16
#define RED_LED 15
#define FAN_LED 27 // Blue LED to simulate a fan when it is hot
#define YELLOW_LED 26

#define CCS811_ADDR 0x5B

const int hours = 14;
const int mins = 37;
const int secs = 30;

const int DARK = 1200;
const int LIGHT = 700;
const int NIGHTTIME = 18;
const int MORNING = 7;

const int WATER_LOW = 500;
const int WATER_NORMAL = 1000;

const int HALF_SEC = 500;
const int ONE_SEC = 1000;
const int FIVE_SECS = 5000;
const int TEN_SECS = 10;

BME280 bme;
CCS811 ccs(CCS811_ADDR);
RTC_DS1307 rtc;

Servo waterPump;

volatile bool waterLowFlag = false; // interrupt

// Interrupt for low water levels 
void IRAM_ATTR waterLevelISR() {
    waterLowFlag = true; // just set the flag
}

typedef struct {
  char msg[64];
} UartMessage;

QueueHandle_t logQ;  // = xQueueCreate(10,sizeof(UartMessage));

void GrowLightManager(void *parameter) {
  DateTime now;
  int light;
  static bool growLightOn = false;

  UartMessage myMsg;
  while (true) {
    now = rtc.now();
    light = analogRead(PHOTO_PIN);
    bool isDay = now.hour() >= MORNING && now.hour() < NIGHTTIME; 

    if(isDay && light > DARK && !growLightOn) {
      //turn on grow lights
      growLightOn = true;
      digitalWrite(WHITE_LED, HIGH);
      //queue message
      strcpy(myMsg.msg, "Grow lights on");
      xQueueSend(logQ, &myMsg, portMAX_DELAY);
    }
    else if((!isDay || light < LIGHT) && growLightOn) {
      //turn off grow lights 
      growLightOn = false; 
      digitalWrite(WHITE_LED, LOW);
      //queue message
      //strcpy(myMsg.msg, "Grow lights off");
      snprintf(myMsg.msg, sizeof(myMsg.msg), "Grow Light: %d and hour: %d", light, now.hour());

      xQueueSend(logQ, &myMsg, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(TEN_SECS));  
  }
}

void WateringManager(void *parameter) {
  int waterLevel;
  static bool pumpOn = false;
  static unsigned long pumpStart = 0;
  const unsigned long PUMP_DURATION = 5000; // 5 seconds 

  UartMessage myMsg;

  while (true) {
    //waterLowFlag = false;
    waterLevel = analogRead(W_LEVEL_PIN);

    if((waterLowFlag || waterLevel < WATER_LOW) && !pumpOn) {
      //use pump to raise water level
      waterLowFlag = false;
      waterPump.write(90);
      pumpOn = true; 
      pumpStart = millis(); 

      //send message in queue
      strcpy(myMsg.msg, "Water Pump Activated");
      xQueueSend(logQ, &myMsg, portMAX_DELAY);
    }

    if(pumpOn){
      waterLevel = analogRead(W_LEVEL_PIN);
      if(millis() - pumpStart >= PUMP_DURATION || waterLevel >= WATER_NORMAL){
        waterPump.write(0);
        pumpOn = false; 

        strcpy(myMsg.msg, "Water Pump Deactivated");
        xQueueSend(logQ, &myMsg, portMAX_DELAY);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(HALF_SEC));  // 0.5 second delay
  }
}

void EnvironmentManager(void *parameter) {
  UartMessage myMsg;

  const float HOT_TEMP = 28.0;
  const float COLD_TEMP = 26.0;
  const float HUMIDITY_MAX = 30.0; // for testing purposes 

  while (true) {

    float temp = bme.readTempC();
    float humidity = bme.readFloatHumidity();
    // float pressure = bme.readFloatPressure() / 100.0;

    // CCS811 
    float eCO2 = 0;
    float TVOC = 0;

    if(ccs.dataAvailable()){
      ccs.readAlgorithmResults();
      eCO2 = ccs.getCO2();
      TVOC = ccs.getTVOC(); 
    }

    if(temp > HOT_TEMP){
      digitalWrite(FAN_LED, HIGH);
      digitalWrite(RED_LED, LOW);
    }
    else if(temp < COLD_TEMP){
      digitalWrite(RED_LED, HIGH);
      digitalWrite(FAN_LED, LOW);
    }
    else{
      digitalWrite(RED_LED, LOW);
      digitalWrite(FAN_LED, LOW);
    }

    // Humidity check
    if (humidity > HUMIDITY_MAX) {
      digitalWrite(YELLOW_LED, HIGH); // turn on LED
      snprintf(myMsg.msg, sizeof(myMsg.msg), "Humidity Warning: %.1f%%", humidity);
      xQueueSend(logQ, &myMsg, portMAX_DELAY);
    } else {
      digitalWrite(YELLOW_LED, LOW); // keep off if in range
      snprintf(myMsg.msg, sizeof(myMsg.msg), "Temp: %.1fC Hum: %.1f%% CO2: %.0f TVOC: %.0f", temp, humidity, eCO2, TVOC);
      xQueueSend(logQ, &myMsg, portMAX_DELAY);
    }
    vTaskDelay(pdMS_TO_TICKS(500));  
  }
}

void LoggerTask(void *parameter) {
  UartMessage recievedMsg;

  while (true) {
    if (xQueueReceive(logQ, &recievedMsg, portMAX_DELAY) == pdPASS) {
      Serial.print("Received: ");
      Serial.println(recievedMsg.msg);
    }
    vTaskDelay(pdMS_TO_TICKS(500));  // 0.5 second delay
  }
}

void setup() {
  pinMode(PHOTO_PIN, INPUT);
  pinMode(W_LEVEL_PIN, INPUT);  
  pinMode(WHITE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(FAN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(WHITE_LED,OUTPUT);

  pinMode(W_LEVEL_PIN, INPUT); // already in your setup
  attachInterrupt(digitalPinToInterrupt(W_LEVEL_PIN), waterLevelISR, FALLING); 

  Serial.begin(115200);
  delay(1000);

  logQ = xQueueCreate(10, sizeof(UartMessage));

  waterPump.attach(SERVO_PIN);
  waterPump.write(0);

  Wire.begin(21, 22); // SDA, SCL

  // Initialize BME280
  if (!bme.begin()) {
    Serial.println("BME280 not detected!");
  } else {
    Serial.println("BME280 initialized!");
  }

  // Initialize CCS
  if (!ccs.begin()) {
    Serial.println("CCS811 not detected!");
  } else {
    Serial.println("CCS811 initialized!");
    ccs.enableInterrupts();
  }

  delay(1000);

  //--- Initialize RTC ---
  if (!rtc.begin()) {
    Serial.println("RTC not detected. Check wiring!");
    while (1)
      ;
  } else {
    Serial.println("RTC initialized");
    rtc.adjust(DateTime(2026, 2, 6, hours, mins, secs));
  }

  //--initialize photo--
  analogReadResolution(12);                 // 0..4095
  analogSetAttenuation(ADC_11db);           // up to ~3.3V range
  analogSetPinAttenuation(W_LEVEL_PIN, ADC_11db);
  analogSetPinAttenuation(PHOTO_PIN, ADC_11db);


  // Creat Grow Light
  xTaskCreate(
    GrowLightManager,    // function
    "GrowLightManager",  // name
    4096,                // stack size (words)
    NULL,                // parameters
    1,                   // priority
    &GrowLightHandler    // task handle
  );                     // core

  //Create Water Manager Task 
  xTaskCreate(
    WateringManager,    // function
    "WateringManager",  // name
    4096,               // stack size (words)
    NULL,               // parameters
    1,                  // priority
    &WateringHandler    // task handle
  );

  // Create Env Manager
  xTaskCreate(
    EnvironmentManager,
    "EnvironmentManager",
    4096,
    NULL,
    1,
    &EnvironmentHandler);

  // Create Logging Task
  xTaskCreate(
    LoggerTask,     // function
    "LoggerTask",   // name
    4096,           // stack size (words)
    NULL,           // parameters
    1,              // priority
    &LoggerHandler  // task handle
  );
}

void loop() {
  // Empty — FreeRTOS tasks run instead
}