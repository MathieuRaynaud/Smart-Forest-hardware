#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <HIH61xx.h>
#include <Wire.h>

#define DEBUG 1 //if you want to print on the serial monitor
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false
// this keeps track of whether we're using the interrupt
// off by default!

// Temperature and Humidity sensor connexion
// A4 (SDA), A5 (SCL)
int transistorTempHumSensor = 9;      // allow you to switch on or off the sensor
HIH61xx<TwoWire> hih(Wire);

// Ozone Sensor connexion
// Sensor <-> Arduino
// TXD <-> 12
// RXD <-> Voltage Diviser Bridge, 13
// 3V3 and GND
SoftwareSerial o3Serial(12, 13); // RX, TX
String dataString = "";
String S_gas;
long H2S;

// GPS connexion :
//   Connect the GPS TX (transmit) pin to Digital 10
//   Connect the GPS RX (receive) pin to Digital 11
SoftwareSerial GPSSerial(10, 11); // RX, TX
Adafruit_GPS GPS(&GPSSerial);
int transistorGPS = 12;      // allow you to switch on or off the GPS
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


int lightSensor = A0;    // select the input pin for the potentiometer
int transistorLightSensor = 13;      // allow you to switch on or off the light sensor
int userOrder = 0;    // User input order on the serial command


void setup() {
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // Initialize all the pin:
  pinMode(lightSensor, INPUT);
  pinMode(transistorLightSensor,OUTPUT);
  digitalWrite(transistorLightSensor,LOW);
  pinMode(transistorGPS,OUTPUT);
  digitalWrite(transistorGPS,LOW);
  pinMode(transistorTempHumSensor,OUTPUT);
  digitalWrite(transistorTempHumSensor,LOW);

  //Ozone Sensor:
  o3Serial.begin(9600); // set the data rate for the SoftwareSerial port
  o3Serial.setTimeout(1000);

  //Humidity and Temperature sensor initialization
  Wire.begin();
  hih.initialise();
  
  if(DEBUG){
    Serial.println("Initialization Done!");
    GPSSerial.println(PMTK_Q_RELEASE);
  }
  
}


void loop() {
  // read the value from the sensor:

  if(userOrder == 1)
  {    
    digitalWrite(transistorLightSensor,HIGH);
    if(DEBUG){
      Serial.println("Light Sensor is on");
    }
    delay(200);
    
    if(DEBUG){
      Serial.print("Light Sensor value : ");
      Serial.println(lightValue());
    }
    userOrder=0;
    digitalWrite(transistorLightSensor,LOW);
  }
  if(userOrder == 2)
  {
    digitalWrite(transistorGPS,HIGH);
    delay(2000);
    if(DEBUG){
      Serial.println("GPS is on");
    }
    
    //Turn on the GPS serial and set the baud rate at 9600, 
  GPS.begin(9600);
  // to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate at 1 Hz 
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
   useInterrupt(true);

   delay(4000);
   if(DEBUG){
      Serial.println("Wait response");
   }
    char c = GPS.read();
      // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences! 
      // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
      //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);    
    while(GPS.fix == 0) {
    }
    
    Serial.print("Location (in degrees, works with Google Maps): ");
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(", "); 
    Serial.println(GPS.longitudeDegrees, 4);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
      
    userOrder=0;
    digitalWrite(transistorGPS,LOW);
    if(DEBUG){
      Serial.println("GPS is off");
   }
  }
  if (userOrder == 3) {
    digitalWrite(transistorTempHumSensor,HIGH);
    delay(500);
    
    while(hih.isSampling()){ }
    hih.start();
    while(!hih.isFinished()) {    
      hih.process();
    }
    Serial.print("RH: "); //transistorTempHumSensor
    Serial.print(hih.getRelHumidity() / 100.0);
    Serial.println(" %");
    Serial.print("Ambient: ");
    Serial.print(hih.getAmbientTemp() / 100.0);
    Serial.println(" deg C");
    Serial.print("Status: ");
    Serial.println(hih.getStatus());
    
    userOrder=0;
    digitalWrite(transistorTempHumSensor,LOW);
    Serial.println("Humidity and Temperature sensor off !");
  }
  if (userOrder == 4) {
    delay(1000);
    o3Serial.print('\r');
    if (o3Serial.available()) {
        dataString = o3Serial.readStringUntil('\n');
        Serial.println("********************************************************************");
        //Serial.println(*dataString);
        int idx1 = dataString.indexOf(',');        
        int idx2 = dataString.indexOf(',', idx1 + 1);
        
        // Hint: after comma there's a space - it should be ignored
        S_gas = dataString.substring(idx1 + 2, idx2);
        H2S = S_gas.toInt();        
        
        Serial.print ("H2S level is ");
        Serial.print (H2S);
        Serial.println (" ppb");
      }
    userOrder=0;
  }
  
}

float lightValue(){
  float sensorValue= 0;
  for (int i=0;i<50;i++) {
    sensorValue += analogRead(lightSensor);
  }
  sensorValue = sensorValue/50.0; //Take the mean of 50 values
  sensorValue = sensorValue*(5.0/1023.0); //Convett into the voltage
  return sensorValue/(10000.0*0.0000005); //Obtain the illuminance (lx) from the collector light current
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

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
