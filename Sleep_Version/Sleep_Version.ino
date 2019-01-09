#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


#define DEBUG 1 //if you want to print on the serial monitor

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
int transistorLightSensor = 13;      // allow you to switch on or off the light sensor
int userOrder = 1;    // User input order on the serial command


// Sleep timer
volatile bool watchdogActivated = false;
int sleepIterations = 0;
#define MAX_SLEEP_ITERATIONS 2

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


  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet
  // around page 50, Watchdog Timer.
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

  //Humidity and Temperature sensor initialization
  Wire.begin();


  if (DEBUG) {
    Serial.println("Initialization Done!");
  }

}


void loop() {
  // read the value from the sensor:

  if (watchdogActivated)
  {
    watchdogActivated = false;
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;
    if (sleepIterations >= MAX_SLEEP_ITERATIONS) {
      // Reset the number of sleep iterations.
      sleepIterations = 0;

      if (userOrder == 1)
      {
        digitalWrite(transistorLightSensor, HIGH);
        if (DEBUG) {
          Serial.println("Light Sensor is on");
        }
        delay(200);

        if (DEBUG) {
          Serial.print("Light Sensor value : ");
          Serial.println(lightValue());
        }
        userOrder = 2 ;
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

        userOrder = 3;
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
        while (Wire.available()) { // slave may send less than requested
          byte c = Wire.read(); // receive a byte as character
          Serial.print(c, HEX);        // print the character
        }
        Serial.println();

        userOrder = 4;
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

        while (userOrder) {
          if (o3Serial.available()) {
            dataString = o3Serial.readStringUntil('\n');
            userOrder = 0;
          }
          else {
            o3Serial.print('\r');
            delay(500);
          }
        }

        int idx1 = dataString.indexOf(',');
        int idx2 = dataString.indexOf(',', idx1 + 1);

        // Hint: after comma there's a space - it should be ignored
        S_gas = dataString.substring(idx1 + 2, idx2);
        H2S = S_gas.toInt();

        userOrder = 1;
        o3Serial.end();
        if (DEBUG) {
          Serial.println("Ozone Sensor is off");
          Serial.print ("H2S level is ");
          Serial.print (H2S);
          Serial.println (" ppb");
        }
      }
    }
  }
  sleep();
}

float lightValue() {
  float sensorValue = 0;
  for (int i = 0; i < 50; i++) {
    sensorValue += analogRead(lightSensor);
  }
  sensorValue = sensorValue / 50.0; //Take the mean of 50 values
  sensorValue = sensorValue * (5.0 / 1023.0); //Convett into the voltage
  return sensorValue / (10000.0 * 0.0000005); //Obtain the illuminance (lx) from the collector light current
}
