#define DEBUG 1 //if you want to print on the serial monitor

int lightSensor = A0;    // select the input pin for the potentiometer
int transistorLightSensor = 13;      // select the pin for the LED
int lightSensorTimeResponse = 200; //wait 200 ms to have the steady value
int userOrder = 0;    // User input order on the serial command


void setup() {
  Serial.begin(9600);
  
  // Initialize all the pin:
  pinMode(lightSensor, INPUT);
  pinMode(transistorLightSensor,OUTPUT);
  digitalWrite(transistorLightSensor,LOW);
  
  
  if(DEBUG){
    Serial.println("Initialization Done!");
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
    delay(lightSensorTimeResponse);
    
    if(DEBUG){
      Serial.print("Light Sensor value : ");
      Serial.println(lightValue());
    }
    userOrder=0;
    digitalWrite(transistorLightSensor,LOW);
  }
  
}

float lightValue(){
  int sensorValue= 0;
  for (int i=0;i<50;i++) {
    sensorValue += analogRead(lightSensor);
  }
  sensorValue = sensorValue/50;
  return (float)(1023-sensorValue)*10/sensorValue;;
}

void serialEvent()
{
  //Reading the user order on the serial only when the user enter something
  if(Serial.available())
  {
    char incomingDuty[5];
    memset(incomingDuty,'\0',5); //initialization of the string 
    int index=0;
    while(Serial.available() > 0)
    {
    //Read character by character all the string written by the user
      incomingDuty[index]=Serial.read(); 
      delay(10);
      index++;
    } 
    
    userOrder=atoi(incomingDuty);
    if(userOrder==1) //in order to prevent any mistake from the user
    {
      if(DEBUG){
        Serial.print("Swicth on the light sensor ! (order= ");
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
