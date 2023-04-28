#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <ThingSpeak.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MNI.h"

//Maximum number of characters
#define SIZE_CHANNEL_ID 30
#define SIZE_API_KEY    50

WiFiManagerParameter channelId("0","Thingspeak Channel ID","",SIZE_CHANNEL_ID);
WiFiManagerParameter apiKey("1","Thingspeak API key","",SIZE_API_KEY);
Preferences preferences; //for accessing ESP32 flash memory

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

//RTOS Handle(s)
TaskHandle_t wifiTaskHandle;
TaskHandle_t nodeTaskHandle;
TaskHandle_t applicationTaskHandle;
TaskHandle_t dataToCloudTaskHandle;
QueueHandle_t nodeToAppQueue;
QueueHandle_t nodeToCloudQueue;

/**
 * @brief
 * @param
 * @param
*/
void ConvStrToInt(char* str, uint32_t* integer)
{
  for(uint32_t i = 0; str[i] != '\0'; i++)
  {
    *integer = *integer * 10 + (str[i] - 48);
  }
}

/**
 * @brief Store new data to specified location in ESP32's flash memory 
 * if the new data is different  from the old data.
*/
static void StoreNewFlashData(const char* flashLoc,const char* newData,
                              const char* oldData,uint8_t dataSize)
{
  if(strcmp(newData,"") && strcmp(newData,oldData))
  {
    preferences.putBytes(flashLoc,newData,dataSize);                              
  }
}

void setup() {
  // put your setup code here, to run once:
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  preferences.begin("AirMonitor", false);
  nodeToAppQueue = xQueueCreate(1,sizeof(SensorData_t));
  nodeToCloudQueue = xQueueCreate(1,sizeof(SensorData_t));
  if(nodeToAppQueue != NULL)
  {
    Serial.println("Node-Application Queue successfully created.");
  }
  else
  {
    Serial.println("Node-Application Queue creation failed.");
  }
  if(nodeToCloudQueue != NULL)
  {
    Serial.println("Node-Cloud Queue successfully creeated");
  }
  else
  {
    Serial.println("Node-Cloud Queue creation failed");
  }
  xTaskCreatePinnedToCore(WiFiManagementTask,"",7000,NULL,1,&wifiTaskHandle,1);
  xTaskCreatePinnedToCore(ApplicationTask,"",30000,NULL,1,&applicationTaskHandle,1);
  xTaskCreatePinnedToCore(NodeTask,"",30000,NULL,1,&nodeTaskHandle,1);
  xTaskCreatePinnedToCore(DataToCloudTask,"",7000,NULL,1,&dataToCloudTaskHandle,1);
}

void loop() {
  // put your main code here, to run repeatedly:

}

/*
 * @brief Manages WiFi configurations (STA and AP modes). Connects
 * to an existing/saved network if available, otherwise it acts as
 * an AP in order to receive new network credentials.
*/
void WiFiManagementTask(void* pvParameters)
{
  const uint16_t accessPointTimeout = 50000; //millisecs
  static WiFiManager wm;
  WiFi.mode(WIFI_STA);  
  wm.addParameter(&channelId);
  wm.addParameter(&apiKey);
  wm.setConfigPortalBlocking(false);
  wm.setSaveParamsCallback(WiFiManagerCallback); 
  //Auto-connect to previous network if available.
  //If connection fails, ESP32 goes from being a station to being an access point.
  Serial.print(wm.autoConnect("AirMonitor")); 
  Serial.println("-->WiFi status");   
  bool accessPointMode = false;
  uint32_t startTime = 0;    
  
  while(1)
  {
    wm.process();
    if(WiFi.status() != WL_CONNECTED)
    {
      if(!accessPointMode)
      {
        if(!wm.getConfigPortalActive())
        {
          wm.autoConnect("AirMonitor"); 
        }
        accessPointMode = true; 
        startTime = millis(); 
      }
      else
      {
        //reset after a timeframe (device shouldn't spend too long as an access point)
        if((millis() - startTime) >= accessPointTimeout)
        {
          Serial.println("\nAP timeout reached, system will restart for better connection");
          vTaskDelay(pdMS_TO_TICKS(1000));
          esp_restart();
        }
      }
    }
    else
    {
      if(accessPointMode)
      {   
        accessPointMode = false;
        Serial.println("Successfully connected, system will restart now");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
      }
    }    
  }
}


void ApplicationTask(void* pvParameters)
{
  LiquidCrystal_I2C lcd(0x27,20,4);
  static SensorData_t sensorData;
  bool isWifiTaskSuspended = false;

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
    /**
     * Suspend Wifi management task if the system is already
     * connected to a WiFi network
     */
    if(WiFi.status() == WL_CONNECTED && !isWifiTaskSuspended)
    {
      vTaskSuspend(wifiTaskHandle);
      Serial.println("WIFI TASK: SUSPENDED");
      isWifiTaskSuspended = true;
    }
    else if(WiFi.status() != WL_CONNECTED && isWifiTaskSuspended)
    {
      vTaskResume(wifiTaskHandle);
      Serial.println("WIFI TASK: RESUMED");
      isWifiTaskSuspended = false;
    }
    //Receives data from Node Task
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
      vTaskSuspend(wifiTaskHandle);
      vTaskSuspend(applicationTaskHandle);
      vTaskSuspend(dataToCloudTaskHandle);
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
      else
      {
        Serial.println("--Failed to send data to Application task\n");
      }
      //Place sensor data in the Node-MQTT Queue
      if(xQueueSend(nodeToCloudQueue,&sensorData,0) == pdPASS)
      {
        Serial.println("--Data successfully sent to cloud task\n");
      }
      else
      {
        Serial.println("--Failed to send data to Cloud task\n");
      }
      vTaskResume(wifiTaskHandle);
      vTaskResume(applicationTaskHandle);
      vTaskResume(dataToCloudTaskHandle);
    } 
  } 
}

void DataToCloudTask(void* pvParameters)
{
  static SensorData_t sensorData;
  static WiFiClient wifiClient;
  ThingSpeak.begin(wifiClient);
  //Previously stored data in ESP32's flash
  char prevChannelId[SIZE_CHANNEL_ID] = {0};
  char prevApiKey[SIZE_API_KEY] = {0};

  preferences.getBytes("0",prevChannelId,SIZE_CHANNEL_ID);
  preferences.getBytes("1",prevApiKey,SIZE_API_KEY); 
  
  uint32_t idInt = 0;
  uint32_t prevUploadTime = millis();

  while(1)
  {
    if(WiFi.status() == WL_CONNECTED)
    {       
      //Receive sensor data from the node-MQTT Queue.
      if(xQueueReceive(nodeToCloudQueue,&sensorData,0) == pdPASS)
      {
        Serial.println("--Cloud Task received data from node task\n");
      }
      //Send data to Thingspeak and MQTT every 20 seconds
      if(millis() - prevUploadTime >= 20000)
      { 
        //Encode data to be sent to Thingspeak
        ThingSpeak.setField(1,sensorData.temp);
        ThingSpeak.setField(2,sensorData.hum);
        ThingSpeak.setField(3,sensorData.NO2);
        ThingSpeak.setField(4,sensorData.NH3);
        ThingSpeak.setField(5,sensorData.CO);
        ThingSpeak.setField(6,sensorData.CO2);
        ThingSpeak.setField(7,sensorData.PM2_5);
        ThingSpeak.setField(8,sensorData.PM10);
        //Convert channel ID from string to Integer
        ConvStrToInt(prevChannelId,&idInt);
        if(ThingSpeak.writeFields(idInt,prevApiKey) == HTTP_CODE_OK)
        {
          Serial.println("SUCCESS: Data sent to ThingspeaK");
        }
        else
        {
          Serial.println("ERROR: Sending to Thingspeak failed");
        }
        prevUploadTime = millis();
      }
    }
  }
}

void WiFiManagerCallback(void)
{
  char prevChannelId[SIZE_CHANNEL_ID] = {0};
  char prevApiKey[SIZE_API_KEY] = {0};
  //Get data stored previously in flash memory
  preferences.getBytes("0",prevChannelId,SIZE_CHANNEL_ID);
  preferences.getBytes("1",prevApiKey,SIZE_API_KEY);
  //Store new data in flash memory if its different from the previously stored ones
  StoreNewFlashData("0",channelId.getValue(),prevChannelId,SIZE_CHANNEL_ID);
  StoreNewFlashData("1",apiKey.getValue(),prevApiKey,SIZE_API_KEY);
}
