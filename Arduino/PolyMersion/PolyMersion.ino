// Import libraries for temperature sensor communication

#include <OneWire.h>
#include <DallasTemperature.h>

/* 
  ===== Pin Definitions =====
  These constants define the pins connected to the pumps, water level switches, 
  temperature sensor bus, and the air valve.
*/


const int pump_1 = 3;      // Pump 1 control pin for filling direction
const int pump_2 = 2;      // Pump 1 control pin for oposite direction, always kept at LOW
const int pump_3 = 10;     // Pump 2 control pin for filling direction
const int pump_4 = 11;     // Pump 2 control pin for oposite direction, always kept at LOW

const int waterLevel = 8;  // Digital input for water level sensor 1
const int waterLevelB = 7; // Digital input for water level sensor 2

const int oneWirePin = 5;  // Digital pin used for OneWire temperature sensor communication

const int valvePin = 9;    // Digital output for the air valve control


// State tracking booleans to avoid repetitive pump start messages.
bool pumpingStarted = false;
bool pumpingStartedB = false;

/* 
  ===== Temperature Sensor Address Setup =====
  These DeviceAddress arrays store the unique addresses for two temperature sensors.
*/
DeviceAddress sensor = {0x28, 0xFF, 0x64, 0x1E, 0x01, 0xD9, 0xA5, 0xBA};   // Sensor 1 address
DeviceAddress sensorB = {0x28, 0xAE, 0x17, 0x46, 0xD4, 0x25, 0x56, 0x2A};  // Sensor 2 address


// Create OneWire bus object and DallasTemperature object for sensor communications.
OneWire oneWireBus(oneWirePin);
DallasTemperature sensors(&oneWireBus);

/*
  ===== Setup Function =====
  This function runs once at startup to configure pin modes and initialize sensors.
*/

void setup() {
  // Start Serial communication for debug and command input.
  Serial.begin(9600);
  
  // Initialize the temperature sensors.
  sensors.begin();
  
  // Set pump control pins as outputs.
  pinMode(pump_1, OUTPUT);
  pinMode(pump_2, OUTPUT);
  pinMode(pump_3, OUTPUT);
  pinMode(pump_4, OUTPUT);
  
  // Set water level sensor pins as inputs.
  pinMode(waterLevel, INPUT);
  pinMode(waterLevelB, INPUT);
  
  // Set the valve control pin as output.
  pinMode(valvePin, OUTPUT); 

  // Initialize pump outputs to stop pumps (0 speed).
  analogWrite(pump_1, 0);
  analogWrite(pump_2, 0);
  analogWrite(pump_3, 0);
  analogWrite(pump_4, 0);
  
  // Activate internal pull-ups for water level sensors.
  // This ensures a default HIGH state when the sensors are inactive.
  digitalWrite(waterLevel, HIGH);
  digitalWrite(waterLevelB, HIGH);
}

/*
  ===== Main Loop Function =====
  The loop() checks water levels, controls pumps accordingly, and also reads serial commands.
*/
void loop() {
  // Read the state of water level sensors (HIGH means water is below the threshold)
  int sensorState = digitalRead(waterLevel); 
  int sensorStateB = digitalRead(waterLevelB);
  
  // Wait for 1 second between readings
  delay(1000); 

  // Buffer to store an incoming serial command (up to 10 characters).
  char cmd[10];                        
                    
  /* 
    ----- Water Level Bath 1 Control -----
    When sensorState is HIGH, it means the water level is below the switch threshold.
    Thus, pump_1 is activated with a set speed (80), that can be adjusted if using a different pump.
    A flag 'pumpingStarted' is used to track pump status.
  */  
  if (sensorState == HIGH) { 
      analogWrite(pump_1, 80);  
      analogWrite(pump_2, 0);
      if (!pumpingStarted) {  
          pumpingStarted = true;
    }
  }
  else {
    // If the water level is sufficient, stop the pump.
    analogWrite(pump_1, 0);
    analogWrite(pump_2, 0);
    pumpingStarted = false;
   }
  /* 
    ----- Water Level Bath 2 Control -----
    When sensorStateB is HIGH, it means the water level is below the switch threshold.
    Thus, pump_2 is activated with a set speed (80), that can be adjusted if using a different pump.
    A flag 'pumpingStartedB' is used to track pump status.
  */  
  if (sensorStateB == HIGH) { 
      analogWrite(pump_3, 80);  
      analogWrite(pump_4, 0);
      if (!pumpingStartedB) {  
          pumpingStartedB = true;
    }
  }
  else {
    // If the water level is sufficient, stop the pump.
    analogWrite(pump_3, 0);
    analogWrite(pump_4, 0);
    pumpingStartedB = false;
   }
 
/* 
    ----- Serial Command Processing -----
    The following code checks if a command was sent via Serial and processes it.
    Supported commands:
      - "PUMP": Activate pump 1.
      - "STOP": Stop pump 1.
      - "TEMP": Read and print temperature from sensor 1.
      - "TEMPB": Read and print temperature from sensor 2.
      - "OPEN VALVE": Open the air valve.
      - "CLOSE VALVE": Close the air valve.
  */
 if (readSerial(cmd)) {
    // Convert the received command to uppercase to make comparisons case-insensitive.
    toUpperCase(cmd);
    if (strstr(cmd, "PUMP")) {                                                       //PUMP ONLY
      analogWrite(pump_1, 80);   //255=full speed, 0=stop, 127=half speed
      analogWrite(pump_2, 0);
    }  
    else if (strstr(cmd, "STOP")) {                                                  //STOP PUMP
      analogWrite(pump_1, 0);
      analogWrite(pump_2, 0);
      //Serial.println("Stop");
    }
    else if (strstr(cmd, "TEMP")) {                                                  //READ TEMPERATURE
      printTemperature();
    }
    else if (strstr(cmd, "TEMPB")) {                                                  //READ TEMPERATURE B
      printTemperatureB();
    }
    else if (strstr(cmd, "OPEN VALVE")) {                                            //OPEN VALVE
      digitalWrite(valvePin, HIGH);
    }
    else if (strstr(cmd, "CLOSE VALVE")) {                                           //CLOSE VALVE
      digitalWrite(valvePin, LOW);
    }
  } 

}
/*
  ===== Helper Function: readSerial =====
  Reads characters from Serial until a newline is encountered, then terminates the string.
  Returns true if a complete command was read, false otherwise.
*/
int i = 0; // Global index for storing incoming characters
bool readSerial(char result[]) {
  while (Serial.available() > 0) {       // Check if there is any data in the Serial buffer
    char inChar = Serial.read();         // Read one character from the Serial buffer
    
    if (inChar == '\n') {                // Check if the character is the newline character
      result[i] = '\0';                  // Terminate the string with a null character
      Serial.flush();                    // Clear the serial transmit buffer (and in older versions,
                                         // it was sometimes used to clear the input - see note below)
      i = 0;                             // Reset index for the next command
      return true;                       // Indicate that a complete command has been read
    }
    // Ignore carriage return characters so they don't clutter the result
    if (inChar != '\r') {
      result[i] = inChar;                // Store the character in the result array
      i++;                               // Increment the index for the next incoming character
    }
    delay(1);                            // Small delay to allow other Serial events
  }
  return false;                          // No complete command was received yet
}

/*
  ===== Helper Function: toUpperCase =====
  Converts a given C-string to uppercase in place.
*/
void toUpperCase(char* str) {
  if (str == NULL) return;
  for (int i = 0; str[i] != '\0'; i++) {
    str[i] = toupper(str[i]);
  }
}

/*
  ===== Helper Function: printTemperature =====
  Requests temperature readings from sensor 1 and prints the temperature in Celsius to Serial.
*/
void printTemperature() {
  sensors.requestTemperatures();
  Serial.print("Temperature at sensor 1: ");
  Serial.print(sensors.getTempC(sensor));
  Serial.println(" ºC");
}
/*
  ===== Helper Function: printTemperatureB =====
  Requests temperature readings from sensor 2 and prints the temperature in Celsius to Serial.
*/
void printTemperatureB() {
  sensors.requestTemperatures();
  Serial.print("Temperature at sensor 2: ");
  Serial.print(sensors.getTempC(sensorB));
  Serial.println(" ºC");
}
