#pragma once

//MNI: Master-Node-Interface
//Handles Serial communication between Master(ESP32) and Node(Nano)
//Features: Bi-directional communication

class MNI
{
  private:
    SoftwareSerial* port;
        
  public:
    MNI(SoftwareSerial* serial, uint32_t baudRate = 9600);
    bool IsReceiverReady(uint8_t expectedNumOfBytes);
    void TransmitData(void * dataBuffer,uint8_t dataSize);
    void ReceiveData(void * dataBuffer,uint8_t dataSize);
};
