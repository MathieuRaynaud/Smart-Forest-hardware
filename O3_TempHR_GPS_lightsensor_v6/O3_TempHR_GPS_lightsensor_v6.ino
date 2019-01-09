#include <SoftwareSerial.h>
#include <Wire.h>
#include <TheThingsNetwork.h>

#define DEBUG 1 //if you want to print on the serial monitor
#define ledPIN 5 //DEBUG led

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
#define pinRST 4
#define LoRaRX 6
#define LoRaTX 7
SoftwareSerial loraSerial(LoRaRX, LoRaTX);
#define freqPlan TTN_FP_EU868
/*
const char *devAddr = "26011974";
const char *nwkSKey = "0FD4DF821371856F6690C46E428FA4A5";
const char *appSKey = "6EDCCF62E148D4CB5070040ADAA6CA66";*/
  const char *devAddr = "f9e02317";
  const char *nwkSKey = "3de6ac4540fca46c0c4a871ff583f30b";
  const char *appSKey = "17d6b299b3f0e5db8fcbca3205ccd588";
TheThingsNetwork ttn(loraSerial, Serial, freqPlan);

// Temperature and Humidity sensor connexion
// A4 (SDA), A5 (SCL)
int deviceAddress = 39;    // device number 39 or 0x27
byte readI2C;

// Ozone Sensor connexion
// Sensor <-> Arduino
// TXD <-> 8
// RXD <-> Voltage Diviser Bridge, 9
// 3V3 and GND
#define o3RXpin 8
#define o3TXpin 9
SoftwareSerial o3Serial(o3RXpin, o3TXpin); // RX, TX
String S_gas;
long H2S;
String dataString = "";



// GPS connexion :
//   Connect the GPS TX (transmit) pin to Digital 10
//   Connect the GPS RX (receive) pin to Digital 11

String Header;
String charFix;
String Latitude;
String DirectionLatitude;
String Longitude;
String DirectionLongitude;
String Altitude;
// variables to split string from GPS serial and ozone sensor serial
byte idx1, idx2, idx3, idx4, idx5, idx6, idx7, idx8, idx9, idx10;


#define lightSensor A0    // select the input pin for the potentiometer
int lightSensorValue = 0; //declare the variable to store the value read by the ADC
byte userOrder = 5;    // User input order on the serial command


void setup() {
  // connect at 9600 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(9600);
  Serial.setTimeout(2000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //Reset RN2483 for init
  pinMode(pinRST, OUTPUT);
  digitalWrite(pinRST, LOW);
  delay(500);
  digitalWrite(pinRST, HIGH);
  delay(2000);


  // Initialize all the pin:
  pinMode(lightSensor, INPUT);

  //Initialization of the software serial pins
  pinMode(o3RXpin, INPUT);
  //pinMode(GPSRXpin, INPUT);
  pinMode(LoRaRX, INPUT);
  pinMode(LoRaTX, OUTPUT);
  pinMode(o3TXpin, OUTPUT);
  //pinMode(GPSTXpin, OUTPUT);


  //Humidity and Temperature sensor initialization
  Wire.begin();

  pinMode(ledPIN,OUTPUT);
  digitalWrite(ledPIN,HIGH);
  delay(500);
  digitalWrite(ledPIN,LOW);
  delay(500);
  digitalWrite(ledPIN,HIGH);
  delay(500);
  digitalWrite(ledPIN,LOW);
  delay(500);

}


void loop() {
  // read the value from the sensor:

  if (userOrder == 1)
  {
    delay(200);
    for (int i = 0; i < 50; i++) {
      lightSensorValue += analogRead(lightSensor);
    }
    lightSensorValue = lightSensorValue / 50; //Take the mean of 50 values

    lightByte[1] = lightSensorValue;
    lightByte[0] = lightSensorValue >> 8;

    userOrder = 2 ;
    digitalWrite(ledPIN,HIGH);
    delay(500);
    digitalWrite(ledPIN,LOW);
    delay(500);
    digitalWrite(ledPIN,HIGH);
    delay(500);
    digitalWrite(ledPIN,LOW);
    delay(500);
  }

  if (userOrder == 2) {

    //GPS initilization
    // start with the data rate and the timeout for the GPS SoftwareSerial port
    Serial.begin(9600);
    Serial.setTimeout(2000);
    // to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    Serial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
    // Set the update rate at 1 Hz
    Serial.println("$PMTK220,1000*1F");
    // Request updates on antenna status, comment out to keep quiet
    //Serial.println("$PGCMD,33,1*6C");

    userOrder = 1; //use the variable as a boolean
    delay(2000);
    //While the position is not found
    while (userOrder == 1) {

      if (Serial.available()) { //if the string is available
        dataString = Serial.readStringUntil('\n'); //read the string from the GPS
        idx1 = dataString.indexOf(',');
        Header = dataString.substring(0, idx1);

        if (Header[4] == 'G') { //in order to select only the $GPGGA frame and not the $GPRMC frame

          idx2 = dataString.indexOf(',', idx1 + 1);
          idx3 = dataString.indexOf(',', idx2 + 1);
          Latitude = dataString.substring(idx2 + 1, idx3); //Recover the Latitude
          idx4 = dataString.indexOf(',', idx3 + 1);
          DirectionLatitude = dataString.substring(idx3 + 1, idx4); //Recover the Direction of Latitude
          idx5 = dataString.indexOf(',', idx4 + 1);
          Longitude = dataString.substring(idx4 + 1, idx5); //Recover the Longitude
          idx6 = dataString.indexOf(',', idx5 + 1);
          DirectionLongitude = dataString.substring(idx5 + 1, idx6); //Recover the Direction of Longitude
          idx7 = dataString.indexOf(',', idx6 + 1);
          charFix = dataString.substring(idx6 + 1, idx7); //Recover the fix quality value
          idx8 = dataString.indexOf(',', idx7 + 1);
          idx9 = dataString.indexOf(',', idx8 + 1);
          idx10 = dataString.indexOf(',', idx9 + 1);
          Altitude = dataString.substring(idx9 + 1, idx10); //Recover the altitude

          if (charFix[0] == '1' || charFix[0] == '2') { //if the GPS have a position
            userOrder = 0;
          }
        }
      }
      else
        delay(200);
    }

    coordToHexa();

    userOrder = 3;
    Serial.end();
    digitalWrite(ledPIN,HIGH);
    delay(500);
    digitalWrite(ledPIN,LOW);
    delay(500);
    digitalWrite(ledPIN,HIGH);
    delay(500);
    digitalWrite(ledPIN,LOW);
    delay(500);
  }

  if (userOrder == 3) {
    delay(500);

    Wire.beginTransmission(deviceAddress);
    Wire.write(78); //start
    Wire.endTransmission();

    delay(100); //Wait for the measurement cycle
    Wire.requestFrom(deviceAddress, 4);    // request 4 bytes from slave device #deviceAddress
    userOrder = 0;
    while (Wire.available()) { // slave may send less than requested
      readI2C = Wire.read(); // receive a byte as character
      tempHRByte[userOrder] = readI2C;
      userOrder++;
    }


    userOrder = 4;
    digitalWrite(ledPIN,HIGH);
    delay(500);
    digitalWrite(ledPIN,LOW);
    delay(500);
    digitalWrite(ledPIN,HIGH);
    delay(500);
    digitalWrite(ledPIN,LOW);
    delay(500);
  }
  if (userOrder == 4) {
    // start with the data rate and the timeout for the Ozone sensor SoftwareSerial port
    o3Serial.begin(9600);
    o3Serial.setTimeout(1000);

    userOrder = 1; //in order to reuse the variable as a boolean


    while (userOrder) {

      if (o3Serial.available()) {
        dataString = o3Serial.readStringUntil('\n');
        idx1 = dataString.indexOf(',');
        idx2 = dataString.indexOf(',', idx1 + 1);

        // Hint: after comma there's a space - it should be ignored
        S_gas = dataString.substring(idx1 + 2, idx2);
        H2S = S_gas.toInt();
        if (H2S != 0) {
          userOrder = 0;
        }
      }
      else {
        o3Serial.print('\r');
        delay(500);
      }
    }

    o3Byte[1] = H2S;
    o3Byte[0] = H2S >> 8;

    userOrder = 5;
    o3Serial.end();
    digitalWrite(ledPIN,HIGH);
    delay(500);
    digitalWrite(ledPIN,LOW);
    delay(500);
    digitalWrite(ledPIN,HIGH);
    delay(500);
    digitalWrite(ledPIN,LOW);
    delay(500);
  }
  if (userOrder == 5) {
    //Start the software serial
    loraSerial.begin(57600);
    //TTN Activation Method : ABP
    ttn.personalize(devAddr, nwkSKey, appSKey);


    //Send data to TTN
    byte data[9];
    data[0] = IdWSByte;
    data[1] = lightByte[0];
    data[2] = lightByte[1];
    data[3] = tempHRByte[0] & 0x3F;
    data[4] = tempHRByte[1];
    data[5] = tempHRByte[2];
    data[6] = tempHRByte[3];
    data[7] = o3Byte[0];
    data[8] = o3Byte[1];
    ttn.sendBytes(data, sizeof(data));

    userOrder = 5;
    delay(5000);
    loraSerial.end();

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

  index = 0;
  altitudeByte[3] = LatiMinutes;
  altitudeByte[2] = LatiMinutes >> 8;
  altitudeByte[1] = LatiMinutes >> 16;
  altitudeByte[0] = LatiDegrees;
  if (DirectionLatitude == "N") {
    altitudeByte[0] = altitudeByte[0] | 1 << 7;
  }

  longByte[4] = LongiMinutes;
  longByte[3] = LongiMinutes >> 8;
  longByte[2] = LongiMinutes >> 16;
  longByte[1] = LongiDegrees;
  longByte[0] = LongiDegrees >> 8;
  if (DirectionLongitude == "E") {
    longByte[0] = longByte[0] | 1 << 7;
  }

  altitudeByte[1] = alti;
  altitudeByte[0] = alti >> 8;

}
