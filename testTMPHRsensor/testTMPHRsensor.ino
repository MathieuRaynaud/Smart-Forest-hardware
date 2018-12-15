// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.
// SDA A4, SDL A5

#include <Wire.h>


int deviceAddress = 39;    // device number 39 or 0x27

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output

  Serial.println("Initialization Done!");
  
}

void loop() {
  
  Serial.print("Ask for humidity value on the device number ");
  Serial.println(deviceAddress);
  
  
  Wire.beginTransmission(deviceAddress);
  Serial.println("Waiting...");
  Wire.write(78); //start  
  Serial.print("endTransmission result: ");
  Serial.println(Wire.endTransmission());

  delay(100); //Wait for the measurement cycle
  
  Serial.print("Read value for device number ");
  Serial.println(deviceAddress);
  
  Wire.requestFrom(deviceAddress, 4);    // request 4 bytes from slave device #deviceAddress
  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read(); // receive a byte as character
    Serial.print(c,HEX);         // print the character
  }
  Serial.println();

  delay(500);
}
