#include <SoftwareSerial.h>
#include <Wire.h>
#include <TheThingsNetwork.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


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
const char *devAddr = "26011B55";
const char *nwkSKey = "F31339A17B955BC04FA7421D579AFC13";
const char *appSKey = "D0D5B702CAB536B55187ABD55A7827E6"; //TTN
/*
  const char *devAddr = "f9e038b9";
  const char *nwkSKey = "3de6ac4540fca46c0c4a871ff583f30b";
  const char *appSKey = "17d6b299b3f0e5db8fcbca3205ccd588"; //LoRa Server*/
  
TheThingsNetwork ttn(loraSerial, Serial, freqPlan); //Declaration of the software serial (RX,TX)
byte data_Sensors[9];  //Payload for the sensor values
byte data_GPS[12];     //Payload for the GPS values


// Temperature and Humidity sensor connexion
// Sensor <-> Arduino
// SDA <-> A4
// SCL <-> A5
int deviceAddress = 39;    // device number 39 or 0x27
byte readI2C;

// Ozone Sensor connexion
// Sensor <-> Arduino
// TXD <-> 8
// RXD <-> Voltage Diviser Bridge <-> 9
// 3V3 and GND
#define o3RXpin 8
#define o3TXpin 9
SoftwareSerial o3Serial(o3RXpin, o3TXpin); //Declaration of the software serial (RX,TX)
String S_gas;
long H2S;
String dataString = "";



// GPS Module connexion
//   Connect the GPS TX (transmit) pin to Digital 10
//   Connect the GPS RX (receive) pin to Digital 11
#define GPSTransistorPin 5
String Header;
String charFix;
String Latitude;
String DirectionLatitude;
String Longitude;
String DirectionLongitude;
String Altitude;
// variables to split string from GPS serial and ozone sensor serial
byte idx1, idx2, idx3, idx4, idx5, idx6, idx7, idx8, idx9, idx10;


//Light Sensor connexion
#define lightSensor A0    // select the input pin for the potentiometer
int lightSensorValue = 0; //declare the variable to store the value read by the ADC
byte userOrder = 1;    // User input order on the serial command


// Sleep parameters
#define MAX_SLEEP_ITERATIONS 2 // Every 450*8s=1h
#define MAX_POS_ITERATIONS 2 // Every 24*1h=24h
int sleepIterations = 0;
int posIterations = 0;
volatile bool watchdogActivated = false;

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.

  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
}


void setup() {
  // connect at 9600 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(9600);
  Serial.setTimeout(2000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  dataString.reserve(100);

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

  //GPS transistor initialization
  pinMode(GPSTransistorPin,OUTPUT);
  digitalWrite(GPSTransistorPin,LOW);


  //Humidity and Temperature sensor initialization
  Wire.begin();

  noInterrupts();

  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1 << WDRF);

  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1 << WDP0) | (1 << WDP3);

  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1 << WDIE);

  // Enable interrupts again.
  interrupts();

  Serial.println(F("Setup complete."));

}


void loop() {


  if (watchdogActivated)
  {
    Serial.println("Awaken");
    watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      Serial.println("Sleep iteration elapsed");
      // Reset the number of sleep iterations.
      sleepIterations = 0;
      // Increase the count of pos iterations
      posIterations +=1;
      // read the value from the sensor:



        /* ######################### Getting light sensor value ####################################*/
        delay(200);
        for (int i = 0; i < 50; i++) {
          lightSensorValue += analogRead(lightSensor);
        }
        lightSensorValue = lightSensorValue / 50; //Take the mean of 50 values

        lightByte[1] = lightSensorValue;
        lightByte[0] = lightSensorValue >> 8;

        /* ######################### Getting GPS values ####################################*/
        /*
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
        */



        /* ######################### Getting temperature and humidity values ####################################*/
        //delay(500) // Setting time if we use transistors to control the power
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

        /* ######################### Getting ozone sensor values ####################################*/
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

        o3Serial.end();
      
        /* ######################### Sending data of sensors through LoRa ####################################*/
        //Start the software serial
        loraSerial.begin(57600);
        //TTN Activation Method : ABP
        ttn.personalize(devAddr, nwkSKey, appSKey);


        //Send data to TTN
        data_Sensors[0] = IdWSByte | 0x80; // if MSB=1 -> Sensors payload
        data_Sensors[1] = lightByte[0];
        data_Sensors[2] = lightByte[1];
        data_Sensors[3] = tempHRByte[0] & 0x3F;
        data_Sensors[4] = tempHRByte[1];
        data_Sensors[5] = tempHRByte[2];
        data_Sensors[6] = tempHRByte[3];
        data_Sensors[7] = o3Byte[0];
        data_Sensors[8] = o3Byte[1];
        ttn.sendBytes(data_Sensors, sizeof(data_Sensors));
        loraSerial.end();

      if (posIterations >= MAX_POS_ITERATIONS) {
         Serial.println("Pos iteration elapsed");

        /* ######################### Getting GPS values ####################################*/
        //GPS initilization
        // start with the data rate and the timeout for the GPS SoftwareSerial port
        digitalWrite(GPSTransistorPin,HIGH);
        delay(500);
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
              Serial.println(dataString);
              if (charFix[0] == '1' || charFix[0] == '2') { //if the GPS have a position
                userOrder = 0;
              }
            }
          }
          else
            delay(200);
        }
        digitalWrite(GPSTransistorPin,LOW);
        coordToHexa();

        userOrder = 3;
        Serial.end();

        /* ######################### Sending GPS data through LoRa ####################################*/
        //Start the software serial
        loraSerial.begin(57600);
        //TTN Activation Method : ABP
        ttn.personalize(devAddr, nwkSKey, appSKey);


        //Send data to TTN
        data_GPS[0] = IdWSByte & 0x7F; // if MSB=0 -> GPS payload
        data_GPS[1] = longByte[0]; //Longitude
        data_GPS[2] = longByte[1];
        data_GPS[3] = longByte[2];
        data_GPS[4] = longByte[3];
        data_GPS[5] = longByte[4];
        data_GPS[6] = latiByte[0]; //Latitude
        data_GPS[7] = latiByte[1];
        data_GPS[8] = latiByte[2];
        data_GPS[9] = latiByte[3];
        data_GPS[10] = altitudeByte[0]; //Altitude
        data_GPS[11] = altitudeByte[1];

        ttn.sendBytes(data_GPS, sizeof(data_GPS));
        loraSerial.end();
        
        
      }
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
