#include <SoftwareSerial.h>

// GPS connexion :
//   Connect the GPS TX (transmit) pin to Digital 10
//   Connect the GPS RX (receive) pin to Digital 11
SoftwareSerial GPSSerial(10, 11);
int transistorGPS = 12;      // allow you to switch on or off the GPS

int userOrder = 1;
String dataString;
String Header;
String charFix;
boolean Fix;
String Latitude;
String DirectionLatitude;
String Longitude;
String DirectionLongitude;
String Altitude;

int DEBUG =1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  
  GPSSerial.begin(9600);
  GPSSerial.setTimeout(2000);
  // to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPSSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  // Set the update rate at 1 Hz 
  GPSSerial.println("$PMTK220,1000*1F");
  // Request updates on antenna status, comment out to keep quiet
  //GPSSerial.println("$PGCMD,33,1*6C");

  Serial.println("Initialization done !!");
}

void loop() {

    while(userOrder) {     
      if (GPSSerial.available()) {
        dataString = GPSSerial.readStringUntil('\n');
        int idx1 = dataString.indexOf(',');
        Header = dataString.substring(0, idx1);        
        if(Header[4]=='G'){ //in order to select only the $GPRMC/$GPGGA string
          int idx2 = dataString.indexOf(',', idx1 + 1);
          int idx3 = dataString.indexOf(',', idx2 + 1);
          Latitude = dataString.substring(idx2 + 1, idx3);
          int idx4 = dataString.indexOf(',', idx3 + 1);
          DirectionLatitude = dataString.substring(idx3 + 1, idx4);
          int idx5 = dataString.indexOf(',', idx4 + 1);
          Longitude = dataString.substring(idx4 + 1, idx5);
          int idx6 = dataString.indexOf(',', idx5 + 1);
          DirectionLongitude = dataString.substring(idx5 + 1, idx6);
          int idx7 = dataString.indexOf(',', idx6 + 1);
          charFix = dataString.substring(idx6 + 1, idx7);
          int idx8 = dataString.indexOf(',', idx7 + 1);
          int idx9 = dataString.indexOf(',', idx8 + 1);
          int idx10 = dataString.indexOf(',', idx9 + 1);
          Altitude = dataString.substring(idx9 + 1, idx10);
          if(charFix[0] == '1'){
            Fix = true;
            Serial.println("position found");
            Serial.print("Latitude is : ");
            Serial.print(Latitude);
            Serial.print(" ");
            Serial.println(DirectionLatitude);
            Serial.print("Longitude is : ");
            Serial.print(Longitude);
            Serial.print(" ");
            Serial.println(DirectionLongitude);
            Serial.print("The Altitude is : ");
            Serial.println(Altitude);
            userOrder=0;
            Serial.println("Salut");
          }
          else {
            Fix = false;
            Serial.println("position not found");
          }          
          Serial.println(dataString);
          Serial.println(charFix[0]);
        }
        
      }
      else
        delay(200);
    }
    //Serial.println(dataString);
    delay(200);
  /*
   if (p[0] == 'A') 
      fix = true;
    else if (p[0] == 'V')
      fix = false;
    else
      return false;
   */
}

//void serialEvent()
//{
//  //Reading the user order on the serial only when the user enter something
//  if(Serial.available())
//  {
//    char incomingOrder[5];
//    memset(incomingOrder,'\0',5); //initialization of the string 
//    int index=0;
//    while(Serial.available() > 0)
//    {
//    //Read character by character all the string written by the user
//      incomingOrder[index]=Serial.read(); 
//      delay(10);
//      index++;
//    } 
//    
//    userOrder=atoi(incomingOrder);
//    if(userOrder==1) //in order to prevent any mistake from the user
//    {
//      if(DEBUG){
//        Serial.print("Swicth on the light sensor ! (order= ");
//        Serial.print(userOrder);
//        Serial.println(")");
//      }
//    }
//    else if (userOrder==2) {
//      if(DEBUG){
//        Serial.print("Swicth on the GPS ! (order= ");
//        Serial.print(userOrder);
//        Serial.println(")");
//      }
//    }
//    else if (userOrder==3) {
//      if(DEBUG){
//        Serial.print("Swicth on the Humidity and Temperature sensor ! (order= ");
//        Serial.print(userOrder);
//        Serial.println(")");
//      }
//    }
//    else if (userOrder==4) {
//      if(DEBUG){
//        Serial.print("Swicth on the Ozone sensor ! (order= ");
//        Serial.print(userOrder);
//        Serial.println(")");
//      }
//    }
//    else {
//      userOrder=0;
//      if(DEBUG){
//        Serial.println("False input");
//      }
//    }
//  }
//  
//}
