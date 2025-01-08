#include <DFRobot_DHT11.h>
#include <OneWire.h>
#include <DallasTemperature.h>

int pump_1 = 3; 
int pump_2 = 2; 
const int waterLevel = 8;
const int oneWirePin = 5;
const int valvePin = 9; 
bool pumpingStarted = false;


OneWire oneWireBus(oneWirePin);
DallasTemperature sensor(&oneWireBus);

DFRobot_DHT11 dht;


void setup() {
  Serial.begin(9600);
  sensor.begin();
  pinMode(pump_1, OUTPUT);
  pinMode(pump_2, OUTPUT);
  pinMode(waterLevel, INPUT);
  pinMode(valvePin, OUTPUT); 
  analogWrite(pump_1, 0);
  analogWrite(pump_2, 0);
  digitalWrite(waterLevel, HIGH);
}


void loop() {

  int sensorState = digitalRead(waterLevel); 
  //Serial.println(sensorState);  
  delay(1000); 

  char cmd[10];                        

  if (sensorState == HIGH) { 
      analogWrite(pump_1, 80);   //255=full speed, 0=stop, 127=half speed
      analogWrite(pump_2, 0);
      if (!pumpingStarted) {  
          Serial.println("Pumping...");
          pumpingStarted = true;
    }
  }
  else {
    analogWrite(pump_1, 0);
    analogWrite(pump_2, 0);
    pumpingStarted = false;
   }
 

 if (readSerial(cmd)) {
    toUpperCase(cmd);
    if (strstr(cmd, "PUMP")) {                                                       //PUMP ONLY
      analogWrite(pump_1, 80);   //255=full speed, 0=stop, 127=half speed
      analogWrite(pump_2, 0);
      Serial.println("Pumping...");
    }  
    else if (strstr(cmd, "STOP")) {                                                  //STOP PUMP
      analogWrite(pump_1, 0);
      analogWrite(pump_2, 0);
      Serial.println("Stop");
    }
    else if (strstr(cmd, "TEMP")) {                                                  //READ TEMPERATURE
      printTemperature();
    }
    else if (strstr(cmd, "OPEN VALVE")) {                                            //OPEN VALVE
      digitalWrite(valvePin, HIGH);
    }
    else if (strstr(cmd, "CLOSE VALVE")) {                                           //CLOSE VALVE
      digitalWrite(valvePin, LOW);
    }
  } 

}

int i = 0;
bool readSerial(char result[]) {
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == '\n') {
      result[i] = '\0';
      Serial.flush();
      i = 0;
      return true;
    }
    if (inChar != '\r') {
      result[i] = inChar;
      i++;
    }
    delay(1);
  }
  return false;
}


void toUpperCase(char* str) {
  if (str == NULL) return;
  for (int i = 0; str[i] != '\0'; i++) {
    str[i] = toupper(str[i]);
  }
}


void printTemperature() {
  //Serial.println("Reading temperatures: ");
  sensor.requestTemperatures();
  Serial.print("Temperature at sensor 1: ");
  Serial.print(sensor.getTempCByIndex(0));
  Serial.println(" ºC");
}