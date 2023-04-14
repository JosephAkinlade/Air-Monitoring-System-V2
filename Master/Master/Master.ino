#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MNI.h"

typedef struct
{
  float temp;
  float hum;
  float NO2;
  float NH3;
  float CO;
  float CO2;
  float PM2_5;
  float PM10;
}SensorData_t;

//RTOS Handles
TaskHandle_t nodeTaskHandle;
TaskHandle_t applicationTaskHandle;
QueueHandle_t nodeToAppQueue;

void setup() {
  // put your setup code here, to run once:
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  nodeToAppQueue = xQueueCreate(1,sizeof(SensorData_t));
  if(nodeToAppQueue != NULL)
  {
    Serial.println("Node-Application Queue successfully created.");
  }
  else
  {
    Serial.println("Node-Application Queue creation failed.");
  }
  xTaskCreatePinnedToCore(ApplicationTask,"",30000,NULL,1,&applicationTaskHandle,1);
  xTaskCreatePinnedToCore(NodeTask,"",30000,NULL,1,&nodeTaskHandle,1);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void ApplicationTask(void* pvParameters)
{
  LiquidCrystal_I2C lcd(0x27,20,4);
  static SensorData_t sensorData;

  //Start-up message
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print(" AIR-MONITOR");
  vTaskDelay(pdMS_TO_TICKS(1500));
  lcd.clear();
  lcd.print("LOADING...");
  vTaskDelay(pdMS_TO_TICKS(1500));
  lcd.clear();
  vTaskResume(nodeTaskHandle);

  //Simple FSM to periodically change parameters displayed
  const uint8_t displayState1 = 0;
  const uint8_t displayState2 = 1;
  uint8_t displayState = displayState1;
  uint32_t prevTime = millis();

  while(1)
  {
    if(xQueueReceive(nodeToAppQueue,&sensorData,0) == pdPASS)
    {
      Serial.println("--Application Task recived data from Node task\n");
    }
    //FSM[Displays the received sensor data received on the LCD]
    switch(displayState)
    {
      case displayState1:
        lcd.setCursor(0,0);
        lcd.print("Temp: ");
        lcd.print(sensorData.temp);
        lcd.print(" C");
        lcd.setCursor(0,1);
        lcd.print("Hum: ");
        lcd.print(sensorData.hum);
        lcd.print(" %");
        lcd.setCursor(0,2);
        lcd.print("NO2 conc: ");
        lcd.print((uint16_t)sensorData.NO2);
        lcd.print(" PPM");
        lcd.setCursor(0,3);
        lcd.print("NH3 conc: ");
        lcd.print((uint16_t)sensorData.NH3);
        lcd.print(" PPM");
        if(millis() - prevTime >= 4000)
        {
          displayState = displayState2;
          prevTime = millis();
          lcd.clear();
        }
        break;

      case displayState2:
        lcd.setCursor(0,0);
        lcd.print("CO conc: ");
        lcd.print(sensorData.CO);
        lcd.print(" PPM");
        lcd.setCursor(0,1);
        lcd.print("CO2 conc: ");
        lcd.print(sensorData.CO2);
        lcd.print(" PPM");
        lcd.setCursor(0,2);
        lcd.print("PM2.5(ug/m3): ");
        lcd.print((uint16_t)sensorData.PM2_5);
        lcd.setCursor(0,3);
        lcd.print("PM10(ug/m3): ");
        lcd.print((uint16_t)sensorData.PM10);
        if(millis() - prevTime >= 4000)
        {
          displayState = displayState1;
          prevTime = millis();
          lcd.clear();
        }
        break;
    }
  }
}

void NodeTask(void* pvParameters)
{
  vTaskSuspend(NULL);
  static MNI mni(&Serial2);
  static SensorData_t sensorData = {0};
  uint32_t prevTime = millis();
  uint8_t query = 0xAA;
  const uint8_t txDataSize = 1;
  const uint8_t rxBufferSize = sizeof(sensorData);
  
  while(1)
  {
    if(millis() - prevTime >= 2500)
    {
      mni.TransmitData(&query,txDataSize);
      prevTime = millis();
    }
    
    if(mni.IsReceiverReady(rxBufferSize))
    {
      vTaskSuspend(applicationTaskHandle);
      mni.ReceiveData(&sensorData,rxBufferSize);
      //Debug
      Serial.print("Temp: ");
      Serial.println(sensorData.temp);
      Serial.print("Hum: ");
      Serial.println(sensorData.hum);
      Serial.print("NO2: ");
      Serial.println((uint16_t)sensorData.NO2);
      Serial.print("NH3: ");
      Serial.println((uint16_t)sensorData.NH3);
      Serial.print("CO: ");
      Serial.println(sensorData.CO);
      Serial.print("CO2: ");
      Serial.println(sensorData.CO2);
      Serial.print("PM 2.5 (ug/m3): ");
      Serial.println((uint16_t)sensorData.PM2_5);
      Serial.print("PM 10.0 (ug/m3): ");
      Serial.println((uint16_t)sensorData.PM10);
      //Place sensor data in Queue
      if(xQueueSend(nodeToAppQueue,&sensorData,0) == pdPASS)
      {
        Serial.println("--Data successfully sent to Application Task");
      }
      vTaskResume(applicationTaskHandle);
    } 
  } 
}
