
  byte lightByte[2];
  byte latiByte[4];
  byte longByte[5];
  byte altitudeByte[2];
  byte o3Byte[2];
  byte tempHRByte[4];
  String Latitude = "4042.6140";
  String DirectionLatitude = "N";
  String Longitude = "00101.4160";
  String DirectionLongitude = "E";
  String Altitude = "110.9";
  int lightSensorValue=889;
  int H2S=321;
  byte rawTMPHRByte = 0x23A654FC;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start!!"); 
  
  Serial.println("");
  Serial.println("");
  Serial.println("");

  lightByte[1]=lightSensorValue;
  lightByte[0]=lightSensorValue >> 8;
  Serial.println("Light Sensor: ");
  Serial.println(lightByte[0],HEX);
  Serial.println(lightByte[1],HEX);
  Serial.println("");
  
  Serial.print("o3 value : ");
  Serial.println(H2S);
  o3Byte[1]=H2S;
  o3Byte[0]=H2S>>8;
  Serial.println("Ozone :");
  Serial.println(o3Byte[0],HEX);
  Serial.println(o3Byte[1],HEX);
  Serial.println("");
  
  coordToHexa();
}

void loop() {
  // put your main code here, to run repeatedly:
  
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
  
  Serial.println(Latitude);
  Serial.print("LatiDegrees : ");
  Serial.println(LatiDegrees);
  Serial.print("LatiMinutes : ");
  Serial.println(LatiMinutes);
  Serial.print("alti : ");
  Serial.println(alti);

  Serial.println();
  Serial.println(Longitude);
  Serial.print("LongiDegrees : ");
  Serial.println(LongiDegrees);
  Serial.print("LongiMinutes : ");
  Serial.println(LongiMinutes);
  Serial.println();
  Serial.println();

  index=0;
  altitudeByte[3] = LatiMinutes;
  altitudeByte[2] = LatiMinutes >> 8;
  altitudeByte[1] = LatiMinutes >> 16;
  altitudeByte[0] = LatiDegrees;
  if(DirectionLatitude == "N"){
    altitudeByte[0] = altitudeByte[0] | 1 << 7;
  }
  Serial.println("Latitude :");
  Serial.println(altitudeByte[0],HEX);
  Serial.println(altitudeByte[1],HEX);
  Serial.println(altitudeByte[2],HEX);
  Serial.println(altitudeByte[3],HEX);
  Serial.println("");
  Serial.println("");

  longByte[4] = LongiMinutes;
  longByte[3] = LongiMinutes >> 8;
  longByte[2] = LongiMinutes >> 16;
  longByte[1] = LongiDegrees;
  longByte[0] = LongiDegrees >> 8;
  if(DirectionLongitude == "E"){
    longByte[0] = longByte[0] | 1 << 7;
  }
  Serial.println("Longitude :");
  Serial.println(longByte[0],HEX);
  Serial.println(longByte[1],HEX);
  Serial.println(longByte[2],HEX);
  Serial.println(longByte[3],HEX);
  Serial.println(longByte[4],HEX);
  Serial.println("");
  Serial.println("");

  altitudeByte[1] = alti;
  altitudeByte[0] = alti >> 8;
  Serial.println("altitude :");
  Serial.println(altitudeByte[0],HEX);
  Serial.println(altitudeByte[1],HEX);
  
}
