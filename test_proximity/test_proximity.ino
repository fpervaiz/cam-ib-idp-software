/*SHARP GP2Y0A21YK0F IR sensor with Arduino and SharpIR library example code. More info: https://www.makerguides.com */
// Include the library(-ies):
#include <SharpIR.h>
#include <RingBuffer.h>

// Define model and input pin:
#define IRPin A7
#define model 1080

// Create variable to store the distances:
double analogread;
double voltage;
double dis;
double avg_dis;

using namespace std;

RingBuffer<double, 6> dis_his;

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
  dis = 27.726*pow(voltage, -1.2045);
  dis_his.push_back(dis);
  
  //Serial.println("Mean distance: %d cm", 1dis);
  delay(1000);

  avg_dis = 0.0;
  for (int i = 0; i < dis_his.size(); i++){
      avg_dis += dis_his[i];
    }
  avg_dis /= dis_his.size();
  Serial.println(avg_dis);
}