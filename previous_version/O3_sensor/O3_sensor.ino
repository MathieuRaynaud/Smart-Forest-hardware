//runs// SPEC H2S Sensor 
 
// Serial number of my sensor
// #define mysensor_serial_no 012017030207
 
// #define portOne_RX_BUFFER_SIZE 256
// #define portOne_TX_BUFFER_SIZE 256
 unsigned char S_humi;
// Sensor values
  // The format of the output is: SN[XXXXXXXXXXXX], PPB [0 : 999999], TEMP [-99:99],
  // RH[0:99], RawSensor[ADCCount], TempDigital, RHDigital, Day[0:99], Hour [0:23]
  // Note that on Arduino Due integer variable (int)  stores a 32-bit (4-byte) value. 
  // This yields a range of -2,147,483,648 to 2,147,483,647 
  // (minimum value of -2^31 and a maximum value of (2^31) - 1). 
  // On 8 bit boards some readings have to be recorded as floats
 
// The big change: we use software serial for H2S sensor
#include <SoftwareSerial.h>
// software serial #1: RX = digital pin 10, TX = digital pin 11
SoftwareSerial portOne(0, 1);
 
 
 
String SensorSerialNo; 
long H2S;
long Temperature;
long RH;
long RawSensor;
long TempDigital;
long RHDigital;
int Days;
int Hours;
int Minutes;
int Seconds;
 
#define command_delay 500
#define start_delay 2500
String dataString = "";
String responseString = "";
boolean dataStringComplete = 0;
char inChar;
 
void setup() {
  Serial.begin(9600);
  Serial.println("H2S sensor demo code!");
  portOne.begin(9600);
  // Normally, data is returned within one second
  portOne.setTimeout(1000);
  // reserve 80 bytes for the dataString
  dataString.reserve(150);
  responseString.reserve(150);
  
  // Wait for sensor 
  delay(500);
  flush_portOne();
  
  // EEPROM dump
  SPEC_dump_EEPROM();
  Serial.println(" ");
  Serial.println("STARTING MEASUREMENTS");
  Serial.println(" ");
}  
 
 
 
void loop() {
  // Do a readout every 10 seconds
  SPEC_Data_read();
  SPEC_parse_data();
  SPEC_print_data();
  delay(10000);
}
 
/* ********************************************************************************
 * This function triggers one measurement and receives the data from the sensor
 **********************************************************************************/
void SPEC_Data_read(){
  // First, we do some initialization
  // dataStringComplete is set as "false", as we don't have any valid data received
  dataStringComplete = 0;
  // Clear the data string
  dataString = "";
  // Now we trigger a measurement
  portOne.print(" ");
  // We wait for the sensor to respond
  dataString = portOne.readStringUntil('\n');
  //Serial.println(dataString);
}
 
/* ********************************************************************************
 * This function takes the received string and upodates sensor data
 **********************************************************************************/
void SPEC_parse_data(){
  // Parses the received dataString
  // Data string is comma separated
  // The format of the output is: SN[XXXXXXXXXXXX], PPB [0 : 999999], TEMP [-99:99],
  // RH[0:99], RawSensor[ADCCount], TempDigital, RHDigital, Day[0:99], Hour [0:23],
  // Minute[0:59], Second[0 : 59]\r\n
  // Take a look also at
  // https://stackoverflow.com/questions/11068450/arduino-c-language-parsing-string-with-delimiter-input-through-serial-interfa
  // We look first for the SN
  int idx1 = dataString.indexOf(',');
  SensorSerialNo = dataString.substring(0, idx1);
  int idx2 = dataString.indexOf(',', idx1 + 1);
  // Hint: after comma there's a space - it should be ignored
  String S_gas = dataString.substring(idx1 + 2, idx2);
  H2S = S_gas.toInt();
  int idx3 = dataString.indexOf(',', idx2 + 1);
  String S_temp = dataString.substring(idx2 + 2, idx3);
  Temperature = S_temp.toInt();
  int idx4 = dataString.indexOf(',', idx3 + 1);
  String S_humi = dataString.substring(idx3 + 2, idx4);
  RH = S_humi.toInt();
  int idx5 = dataString.indexOf(',', idx4 + 1);
  String S_raw_gas = dataString.substring(idx4 + 2, idx5);
  RawSensor = S_raw_gas.toInt();
  int idx6 = dataString.indexOf(',', idx5 + 1);
  String S_Tdigital = dataString.substring(idx5 + 2, idx6);
  TempDigital = S_Tdigital.toInt();
  int idx7 = dataString.indexOf(',', idx6 + 1);
  String S_RHdigital = dataString.substring(idx6 + 2, idx7);
  RHDigital = S_RHdigital.toInt();
  int idx8 = dataString.indexOf(',', idx7 + 1);
  String S_Days = dataString.substring(idx7 + 2, idx8);
  Days = S_Days.toInt();
  int idx9 = dataString.indexOf(',', idx8 + 1);
  String S_Hours = dataString.substring(idx8 + 2, idx9);
  Hours = S_Hours.toInt();
  int idx10 = dataString.indexOf(',', idx9 + 1);
  String S_Minutes = dataString.substring(idx9 + 2, idx10);
  Minutes = S_Minutes.toInt();
  int idx11 = dataString.indexOf('\r');
  String S_Seconds = dataString.substring(idx10 + 2, idx11);
  Seconds = S_Seconds.toInt();
}
 
 
/* ********************************************************************************
 * This function prints the sensor data
 **********************************************************************************/
void SPEC_print_data(){
  Serial.println("********************************************************************");
  Serial.print ("Sensor Serial No. is ");
  Serial.println (S_humi);
  Serial.print ("H2S level is ");
  Serial.print (H2S);
  Serial.println (" ppb");

}
 
 
/* ********************************************************************************
 * EEPROM dump
 **********************************************************************************/
void SPEC_dump_EEPROM(){
  // First we trigger a measurement
  portOne.print(" ");
  // Within one second time we send the command "e"
  delay(400);
  portOne.print("e");
  dataString = portOne.readStringUntil('\n');
  // You can uncomment this line if you wish
  //Serial.println(dataString);
  for (int i=0; i<20; i++){ 
    responseString = portOne.readStringUntil('\n');
    Serial.println(responseString);
  }   
}  
 
void flush_portOne(){
  // Do we have data in the serial buffer?
  // If so, flush it
  if (portOne.available() > 0){
    Serial.println ("Flushing serial buffer...");
    while(1){
      inChar = (char)portOne.read();
      delay(10);
      Serial.print(inChar);
      if (inChar == '\n') break; 
    }
    Serial.println (" ");
    Serial.println ("Buffer flushed!");
  }
}
