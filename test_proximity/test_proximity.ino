/*SHARP GP2Y0A21YK0F IR sensor with Arduino and SharpIR library example code. More info: https://www.makerguides.com */
// Include the library:
#include <SharpIR.h>
// Define model and input pin:
#define IRPin A7
#define model 1080
// Create variable to store the distance:
int analogread;
int voltage;
int distance;
/* Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/

// Create a new instance of the SharpIR class:
//SharpIR mySensor = SharpIR(IRPin, model);
void setup() {
  Serial.begin(9600);
  pinMode (IRPin, INPUT);
}

void loop() {
  analogread = analogRead(IRPin);
  voltage = analogread*5/1024;
  distance = 27.726*pow(voltage, -1.2045);
  Serial.print("Mean distance: ");
  Serial.print(voltage);
  Serial.println(" cm");
  delay(1000);
}