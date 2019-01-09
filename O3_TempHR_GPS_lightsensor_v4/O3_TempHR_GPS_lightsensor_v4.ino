#include <SoftwareSerial.h>
#include <Wire.h>

#define DEBUG 1 //if you want to print on the serial monitor

// Payload to send the data thanks to LoRa network
  byte lightByte[2];
  byte latiByte[4];
  byte longByte[5];
  byte altitudeByte[2];
  byte o3Byte[2];
  byte tempHRByte[4];

// Temperature and Humidity sensor connexion
// A4 (SDA), A5 (SCL)
int transistorTempHumSensor = 12;      // allow you to switch on or off the sensor
int deviceAddress = 39;    // device number 39 or 0x27

// Ozone Sensor connexion
// Sensor <-> Arduino
// TXD <-> 8
// RXD <-> Voltage Diviser Bridge, 9
// 3V3 and GND
int o3RXpin = 8;
int o3TXpin = 9;
SoftwareSerial o3Serial(o3RXpin, o3TXpin); // RX, TX
String S_gas;
long H2S;
String dataString = "";



// GPS connexion :
//   Connect the GPS TX (transmit) pin to Digital 10
//   Connect the GPS RX (receive) pin to Digital 11
int GPSRXpin = 10;
int GPSTXpin = 11;
SoftwareSerial GPSSerial(GPSRXpin, GPSTXpin); // RX, TX
int transistorGPS = 7;      // allow you to switch on or off the GPS
String Header;
String charFix;
String Latitude;
String DirectionLatitude;
String Longitude;
String DirectionLongitude;
String Altitude;


int lightSensor = A0;    // select the input pin for the potentiometer
int lightSensorValue = 0; //declare the variable to store the value read by the ADC
int transistorLightSensor = 13;      // allow you to switch on or off the light sensor
int userOrder = 1;    // User input order on the serial command


void setup() {
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // Initialize all the pin:
  pinMode(lightSensor, INPUT);
  pinMode(transistorLightSensor, OUTPUT);
  digitalWrite(transistorLightSensor, LOW);
  pinMode(transistorGPS, OUTPUT);
  digitalWrite(transistorGPS, LOW);
  pinMode(transistorTempHumSensor, OUTPUT);
  digitalWrite(transistorTempHumSensor, LOW);

  //Initialization of the software serial pins
  pinMode(o3RXpin, INPUT);
  pinMode(GPSRXpin, INPUT);
  pinMode(o3TXpin, OUTPUT);
  pinMode(GPSTXpin, OUTPUT);


  //Humidity and Temperature sensor initialization
  Wire.begin();

  if (DEBUG) {
    Serial.println("Initialization Done!");
  }

}


void loop() {
  // read the value from the sensor:

  if (userOrder == 1)
  {
    digitalWrite(transistorLightSensor, HIGH);
    if (DEBUG) {
      Serial.println("Light Sensor is on");
    }
    delay(200);
    for (int i = 0; i < 50; i++) {
      lightSensorValue += analogRead(lightSensor);
    }
    lightSensorValue = lightSensorValue / 50; //Take the mean of 50 values

    lightByte[1]=lightSensorValue;
    lightByte[0]=lightSensorValue >> 8;
    
    if (DEBUG) {
      Serial.print("Light Sensor value : ");
      Serial.println(lightSensorValue);
      Serial.println(lightByte[0],HEX);
      Serial.println(lightByte[1],HEX);
      Serial.println("");
      Serial.println("");
      Serial.println("");
    }
    userOrder = 0 ;
    digitalWrite(transistorLightSensor, LOW);
  }

  if (userOrder == 2) {

    //GPS initilization
    // start with the data rate and the timeout for the GPS SoftwareSerial port
    GPSSerial.begin(9600);
    GPSSerial.setTimeout(2000);
    // to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPSSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
    // Set the update rate at 1 Hz
    GPSSerial.println("$PMTK220,1000*1F");
    // Request updates on antenna status, comment out to keep quiet
    //GPSSerial.println("$PGCMD,33,1*6C");

    userOrder = 1; //use the variable as a boolean
    digitalWrite(transistorGPS, HIGH);
    delay(2000);
    if (DEBUG) {
      Serial.println("GPS is on");
    }
    //While the position is not found
    while (userOrder) {

      if (GPSSerial.available()) { //if the string is available
        dataString = GPSSerial.readStringUntil('\n'); //read the string from the GPS
        int idx1 = dataString.indexOf(',');
        Header = dataString.substring(0, idx1);

        if (Header[4] == 'G') { //in order to select only the $GPGGA frame and not the $GPRMC frame

          int idx2 = dataString.indexOf(',', idx1 + 1);
          int idx3 = dataString.indexOf(',', idx2 + 1);
          Latitude = dataString.substring(idx2 + 1, idx3); //Recover the Latitude
          int idx4 = dataString.indexOf(',', idx3 + 1);
          DirectionLatitude = dataString.substring(idx3 + 1, idx4); //Recover the Direction of Latitude
          int idx5 = dataString.indexOf(',', idx4 + 1);
          Longitude = dataString.substring(idx4 + 1, idx5); //Recover the Longitude
          int idx6 = dataString.indexOf(',', idx5 + 1);
          DirectionLongitude = dataString.substring(idx5 + 1, idx6); //Recover the Direction of Longitude
          int idx7 = dataString.indexOf(',', idx6 + 1);
          charFix = dataString.substring(idx6 + 1, idx7); //Recover the fix quality value
          int idx8 = dataString.indexOf(',', idx7 + 1);
          int idx9 = dataString.indexOf(',', idx8 + 1);
          int idx10 = dataString.indexOf(',', idx9 + 1);
          Altitude = dataString.substring(idx9 + 1, idx10); //Recover the altitude

          if (charFix[0] == '1') { //if the GPS have a position
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
            userOrder = 0;
          }
        }

      }
      else
        delay(200);
    }

    coordToHexa();
    
    userOrder = 0;
    GPSSerial.end();
    digitalWrite(transistorGPS, LOW);
    if (DEBUG) {
      Serial.println("GPS is off");
    }
  }

  if (userOrder == 3) {
    digitalWrite(transistorTempHumSensor, HIGH);
    delay(500);

    Wire.beginTransmission(deviceAddress);
    Wire.write(78); //start
    Wire.endTransmission();

    delay(100); //Wait for the measurement cycle
    Wire.requestFrom(deviceAddress, 4);    // request 4 bytes from slave device #deviceAddress
    userOrder=0;
    while (Wire.available()) { // slave may send less than requested
      byte c = Wire.read(); // receive a byte as character
      tempHRByte[userOrder]=c;
      userOrder++;
      Serial.print(c,HEX);         // print the character
    }
    Serial.println();
    Serial.println(tempHRByte[0],HEX);
    Serial.println(tempHRByte[1],HEX);
    Serial.println(tempHRByte[2],HEX);
    Serial.println(tempHRByte[3],HEX);
    

    userOrder = 0;
    digitalWrite(transistorTempHumSensor, LOW);
    if (DEBUG) {
      Serial.println("Humidity and Temperature sensor off !");
    }
  }
  if (userOrder == 4) {
    if (DEBUG) {
      Serial.println("Ozone sensor is on !");
    }
    // start with the data rate and the timeout for the Ozone sensor SoftwareSerial port
    o3Serial.begin(9600);
    o3Serial.setTimeout(1000);

    userOrder = 1; //in order to reuse the variable as a boolean
    
    
    while(userOrder) {
      
      if (o3Serial.available()) {
        dataString = o3Serial.readStringUntil('\n');
        int idx1 = dataString.indexOf(',');    
        int idx2 = dataString.indexOf(',', idx1 + 1);
        
        // Hint: after comma there's a space - it should be ignored
        S_gas = dataString.substring(idx1 + 2, idx2);
        H2S = S_gas.toInt();
        if(H2S!=0){
          userOrder=0;
        }
      }
      else {
        o3Serial.print('\r');
        delay(500);
      }
    }
    
    o3Byte[1]=H2S;
    o3Byte[0]=H2S>>8;
        
    userOrder = 0;
    o3Serial.end();
    if (DEBUG) {
      Serial.println("Ozone Sensor is off");
      Serial.print ("H2S level is ");
      Serial.print (dataString);
      Serial.println (" ppb");
      Serial.println("Payload : ");
      Serial.println(o3Byte[0],HEX);
      Serial.println(o3Byte[1],HEX);
      Serial.println("");
      Serial.println("");
    }
  }

}

void coordToHexa() {
  
  int index = 0;
  
  String tmp = "";
  tmp.concat(Latitude[0]);
  tmp.concat(Latitude[1]);
  int LatiDegrees = tmp.toInt();
  
  tmp = "";
  tmp.concat(Longitude[0]);
  tmp.concat(Longitude[1]);
  tmp.concat(Longitude[2]);
  int LongiDegrees = tmp.toInt();

  tmp = "";
  tmp.concat(Latitude[2]);
  tmp.concat(Latitude[3]);
  tmp.concat(Latitude[5]);
  tmp.concat(Latitude[6]);
  tmp.concat(Latitude[7]);
  tmp.concat(Latitude[8]);
  long LatiMinutes = tmp.toInt();
  
  tmp = "";
  tmp.concat(Longitude[3]);
  tmp.concat(Longitude[4]);
  tmp.concat(Longitude[6]);
  tmp.concat(Longitude[7]);
  tmp.concat(Longitude[8]);
  tmp.concat(Longitude[9]);
  long LongiMinutes = tmp.toInt();

  long alti = Altitude.toInt();
  
  Serial.println(Latitude);
  Serial.print("LatiDegrees : ");
  Serial.println(LatiDegrees);
  Serial.print("LatiMinutes : ");
  Serial.println(LatiMinutes);
  Serial.print("alti : ");
  Serial.println(alti);

  Serial.println();
  Serial.println(Longitude);
  Serial.print("LongiDegrees : ");
  Serial.println(LongiDegrees);
  Serial.print("LongiMinutes : ");
  Serial.println(LongiMinutes);
  Serial.println();
  Serial.println();

  index=0;
  altitudeByte[3] = LatiMinutes;
  altitudeByte[2] = LatiMinutes >> 8;
  altitudeByte[1] = LatiMinutes >> 16;
  altitudeByte[0] = LatiDegrees;
  if(DirectionLatitude == "N"){
    altitudeByte[0] = altitudeByte[0] | 1 << 7;
  }
  Serial.println(altitudeByte[0],BIN);
  Serial.println(altitudeByte[1],BIN);
  Serial.println(altitudeByte[2],BIN);
  Serial.println(altitudeByte[3],BIN);
  Serial.println("");
  Serial.println("");

  longByte[4] = LongiMinutes;
  longByte[3] = LongiMinutes >> 8;
  longByte[2] = LongiMinutes >> 16;
  longByte[1] = LongiDegrees;
  longByte[0] = LongiDegrees >> 8;
  if(DirectionLongitude == "E"){
    longByte[0] = longByte[0] | 1 << 7;
  }
  Serial.println(longByte[0],BIN);
  Serial.println(longByte[1],BIN);
  Serial.println(longByte[2],BIN);
  Serial.println(longByte[3],BIN);
  Serial.println(longByte[4],BIN);
  Serial.println("");
  Serial.println("");

  altitudeByte[1] = alti;
  altitudeByte[0] = alti >> 8;
  Serial.println(altitudeByte[0],BIN);
  Serial.println(altitudeByte[1],BIN);
  
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
