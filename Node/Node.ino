#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <PMS.h>
#include <MQ135.h>
#include "MNI.h"
#include "mics6814.h"
#include "MQ7.h"

#define BME280_ADDR 0x76

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

namespace Pin
{
  const uint8_t NO2Pin = A0;
  const uint8_t NH3Pin = A1;
  const uint8_t COPin = A2;
  const uint8_t MQ7Sensor = A3;
  const uint8_t MQ135Pin = A6;
  const uint8_t nodeRx = 6;
  const uint8_t nodeTx = 7;
  const uint8_t pmsTx = 9;
  const uint8_t pmsRx = 10;
};

//Object Instances of Sensors
Adafruit_BME280 bmeSensor;
MICS6814 micsSensor(Pin::NO2Pin,Pin::NH3Pin,Pin::COPin);
SoftwareSerial nodeSerial(Pin::nodeRx,Pin::nodeTx);
SoftwareSerial pmsSerial(Pin::pmsTx,Pin::pmsRx);
PMS pms(pmsSerial);
PMS::DATA pmsData;
MNI mni(&nodeSerial);
MQ7 mq7(Pin::MQ7Sensor);
MQ135 mq135(Pin::MQ135Pin);

SensorData_t sensorData = {0};
uint8_t rxBuffer = 0;
static void Get_SensorData(SensorData_t& data);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Air Monitoring System...");
  pmsSerial.begin(9600);
  bmeSensor.begin(BME280_ADDR);
}

void loop() 
{
  nodeSerial.listen();
  if(mni.IsReceiverReady(1))
  {
    mni.ReceiveData(&rxBuffer,1);
    if(rxBuffer == 0xAA)
    {
      Serial.println("--Query Received");
      Get_SensorData(sensorData); 
      //Debug
      Serial.print("Temp: ");
      Serial.println(sensorData.temp);
      Serial.print("Hum: ");
      Serial.println(sensorData.hum);
      Serial.print("NO2: ");
      Serial.println(sensorData.NO2);
      Serial.print("NH3: ");
      Serial.println(sensorData.NH3);
      Serial.print("CO: ");
      Serial.println(sensorData.CO);
      Serial.print("CO2: ");
      Serial.println(sensorData.CO2);
      Serial.print("PM 2.5 (ug/m3): ");
      Serial.println(sensorData.PM2_5);
      Serial.print("PM 10.0 (ug/m3): ");
      Serial.println(sensorData.PM10);  
      mni.TransmitData(&sensorData,sizeof(sensorData));
    }  
  }
}

void Get_SensorData(SensorData_t& data)
{
  data.temp = bmeSensor.readTemperature();
  data.hum = bmeSensor.readHumidity();
  data.NO2 = micsSensor.GetValue(MICS6814::GAS::NO2);
  data.NH3 = micsSensor.GetValue(MICS6814::GAS::NH3);
  data.CO = mq7.GetPPM();
  data.CO2 = mq135.getPPM() * 100.0;
  pmsSerial.listen();
  pms.requestRead();
  if(pms.readUntil(pmsData))
  {
    data.PM2_5 = pmsData.PM_AE_UG_2_5;
    data.PM10 = pmsData.PM_AE_UG_10_0;
  }
}
