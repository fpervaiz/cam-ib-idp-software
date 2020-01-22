/*Module: ultrasound sensor: SN36696*/
#include <Adafruit_MotorShield.h>

#define microPin 4

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(1);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(2);

long time = 0;
long debounce = 200;
int state = HIGH;
int hitWall;
int previous = LOW;

void setup() {
  Serial.begin (9600);
  pinMode(microPin, INPUT);
  L_MOTOR->run(FORWARD);
  R_MOTOR->run(FORWARD);
}

void hitAWall(){
  L_MOTOR->run(BACKWARD); R_MOTOR->run(BACKWARD);
  for(int i = 150; i>0; i--){
    L_MOTOR->setSpeed(i); R_MOTOR->setSpeed(i);
    }
  R_MOTOR->run(FORWARD);
  for(int i = 150; i>0; i--){
    L_MOTOR->setSpeed(i); R_MOTOR->setSpeed(i);
    }
}

void loop() {

  L_MOTOR->setSpeed(100); R_MOTOR->setSpeed(100);
  
  hitWall = digitalRead(microPin);
  
  if (hitWall == HIGH && previous == LOW && millis() - time > debounce){
    hitAWall(); }

  previous = hitWall;
    
}