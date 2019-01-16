/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInput
*/

int lightSensor = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(lightSensor, INPUT);
  
  Serial.begin(9600);
  
}

void loop() {
  // read the value from the sensor:
  
Serial.println(lightValue());
  
}

float lightValue(){
  int sensorValue= 0;
  for (int i=0;i<50;i++) {
    sensorValue += analogRead(lightSensor);
  }
  sensorValue = sensorValue/50;
  return (float)(1023-sensorValue)*10/sensorValue;;
}
