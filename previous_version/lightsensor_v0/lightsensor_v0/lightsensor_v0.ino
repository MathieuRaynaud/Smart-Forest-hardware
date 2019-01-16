#define LIGHTSENSORPIN A0 //Ambient light sensor reading 
float sensorValue;
float lightValue;


void setup() {
  pinMode(LIGHTSENSORPIN,  INPUT);  
  Serial.begin(9600);
}

void loop() {
  //float reading = analogRead(LIGHTSENSORPIN); //Read light level
  //float square_ratio = reading / 1023.0;      //Get percent of maximum value (1023)
  //square_ratio = pow(square_ratio, 2.0);      //Square to make response more obvious
  //analogWrite(LEDPIN, 255.0 * square_ratio);  //Adjust LED brightness relatively
   for (int i=0;i<50;i++) {
    sensorValue += analogRead(LIGHTSENSORPIN);
  }
  sensorValue = sensorValue/50.0;
  sensorValue = sensorValue*(5.0/1023.0);
  lightValue = sensorValue/(10000.0*0.0000005);
  Serial.print("ADC : ");
  Serial.print(sensorValue);  //Display reading in serial monitor
  Serial.print(" | Light : ");
  Serial.print(lightValue);
  Serial.println(" lx");
}

