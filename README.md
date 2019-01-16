# Smart-Forest-hardware

All the Arduino program for the weather stations. All the pin connection is explained in the comments of the Arduino program. You have also all the test programs for the different sensors and also all the previous version of the weather station program.

## The weather stations use the following sensors:
- **Temperature and Humidity sensor** which uses I2C communication.

- **Ozone sensor** which uses UART communication.

- **Light sensor** which uses the ADC of the Arduino (AO).

- **GPS module** which uses UART communication.

- **LoRa chip (RN2483)** which uses UART communication.


 ## The final Version of the weather station Arduino program includes:
- **Sensors and GPS position measurement, management of the 3 software serials.**

- **Sleep implementation thanks to the watchdog and counters.**

- **Preparation and transmission of the two payloads (sensor values and GPS position).**



