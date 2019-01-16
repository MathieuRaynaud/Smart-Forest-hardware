/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
 #define DEBUG 1 //if you want to print on the serial monitor
#include <SoftwareSerial.h>
// Sensor <-> Arduino 
// TXD <-> 12
// RXD <-> Voltage Diviser Bridge, 13
// 3V3 and GND
SoftwareSerial o3Serial(8,9); // RX, TX
String dataString = "";
String SensorSerialNo; 
String S_gas;
long H2S;
int userOrder = 0;    // User input order on the serial command

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  dataString.reserve(50);
  Serial.println("Start Sensor");

  // set the data rate for the SoftwareSerial port
  o3Serial.begin(9600);
  o3Serial.setTimeout(1000);
  
}

void loop() { // run over and over  
  if(userOrder == 4) {

    /*
    o3Serial.print('\r');
  while(!o3Serial.available()) {}
    //Serial.write(o3Serial.read());
    dataString = o3Serial.readStringUntil('\n'); */
    userOrder=1;
    while(userOrder) {
      int test=o3Serial.available();
      Serial.println(test);
      if (test) {
        dataString = o3Serial.readStringUntil('\n');
        userOrder=0;
      }
      else {
        o3Serial.print('\r');
        delay(500);
      }
    }
    Serial.println("********************************************************************");
    Serial.println(dataString);
    int idx1 = dataString.indexOf(',');    
    int idx2 = dataString.indexOf(',', idx1 + 1);
    
    // Hint: after comma there's a space - it should be ignored
    S_gas = dataString.substring(idx1 + 2, idx2);
    H2S = S_gas.toInt();
    
    
    Serial.print ("H2S level is ");
    Serial.print (H2S);
    Serial.println (" ppb");
    userOrder=0;
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
    else {
      userOrder=0;
      if(DEBUG){
        Serial.println("False input");
      }
    }
  }
  
}
