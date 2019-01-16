#include <SoftwareSerial.h>
#include <Wire.h>
#include <TheThingsNetwork.h>

#define DEBUG 1 //if you want to print on the serial monitor

// Id of the Weather Station :
byte IdWSByte = 0x01;

// Payload to send the data thanks to LoRa network
  byte lightByte[2];
  byte latiByte[4];
  byte longByte[5];
  byte altitudeByte[2];
  byte o3Byte[2];
  byte tempHRByte[4];

// LoRa settings :
#define LoRaRX 6
#define LoRaTX 7
#define pinRST 4
SoftwareSerial loraSerial(LoRaRX,LoRaTX);
#define freqPlan TTN_FP_EU868
const char *devAddr = "26011974";
const char *nwkSKey = "0FD4DF821371856F6690C46E428FA4A5";
const char *appSKey = "6EDCCF62E148D4CB5070040ADAA6CA66";
TheThingsNetwork ttn(loraSerial, Serial, freqPlan);
byte userOrder = 1;    // User input order on the serial command


void setup() {
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Reset RN2483 for init
  pinMode(pinRST, OUTPUT);
  digitalWrite(pinRST, LOW);
  delay(500);
  digitalWrite(pinRST, HIGH);
  delay(2000);

  //Initialization of the software serial pins
  pinMode(LoRaRX, INPUT);
  pinMode(LoRaTX, OUTPUT);

  if (DEBUG) {
    Serial.println("Initialization Done!");
  }

}


void loop() {
  // read the value from the sensor:

  if (userOrder == 5){
    //Start the software serial
    loraSerial.begin(57600);
    //TTN Activation Method : ABP
    ttn.personalize(devAddr, nwkSKey, appSKey);

  
    //Send data to TTN
    byte data[9];
    data[0] = IdWSByte;
    data[1] = lightByte[0];
    data[2] = lightByte[1];
    data[3] = tempHRByte[0];
    data[4] = tempHRByte[1];
    data[5] = tempHRByte[2];
    data[6] = tempHRByte[3];
    data[7] = o3Byte[0];
    data[8] = o3Byte[1];
    ttn.sendBytes(data, sizeof(data));
    
    if(DEBUG){
      Serial.println(data[0],HEX);
      Serial.println(data[1],HEX);
      Serial.println(data[2],HEX);
      Serial.println(data[3],HEX);
      Serial.println(data[4],HEX);
      Serial.println(data[5],HEX);
      Serial.println(data[6],HEX);
      Serial.println(data[7],HEX);
      Serial.println(data[8],HEX);
    }
    
    

    userOrder=0;
    loraSerial.end();
  
  }

}


void serialEvent()
{
  //Reading the user order on the serial only when the user enter something
  if(Serial.available())
  {
    char incomingOrder[5];
    memset(incomingOrder,'\0',5); //initialization of the string 
    int index=0;
    while(Serial.available() > 0)
    {
    //Read character by character all the string written by the user
      incomingOrder[index]=Serial.read(); 
      delay(10);
      index++;
    } 
    
    userOrder=atoi(incomingOrder);
    if(userOrder==1) //in order to prevent any mistake from the user
    {
      if(DEBUG){
        Serial.print("Swicth on the light sensor ! (order= ");
        Serial.print(userOrder);
        Serial.println(")");
      }
    }
    else if (userOrder==2) {
      if(DEBUG){
        Serial.print("Swicth on the GPS ! (order= ");
        Serial.print(userOrder);
        Serial.println(")");
      }
    }
    else if (userOrder==3) {
      if(DEBUG){
        Serial.print("Swicth on the Humidity and Temperature sensor ! (order= ");
        Serial.print(userOrder);
        Serial.println(")");
      }
    }
    else if (userOrder==4) {
      if(DEBUG){
        Serial.print("Swicth on the Ozone sensor ! (order= ");
        Serial.print(userOrder);
        Serial.println(")");
      }
    }
    else if (userOrder==5) {
      if(DEBUG){
        Serial.print("Swicth on the LoRa antenna ! (order= ");
        Serial.print(userOrder);
        Serial.println(")");
      }
    }
    else {
      userOrder=0;
      if(DEBUG){
        Serial.println("False input");
      }
    }
  }
  
}
