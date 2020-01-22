#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M1 & M2 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(1);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(2);

//not used, testing acceleration
// int accelTime = 200;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup(void)
{
  //while (!Serial);// For use only while plugged into USB an with Serial Monitor open
  AFMS.begin();  // create with the default frequency 1.6KHz

  // turn off both motors
  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
    
  Serial.begin(115200);
  Serial.println(F("Robot Drive Test"));
  Serial.println(F("-----------------------------------------")); 

}

void loop(void)
{
  uint8_t i;
  
  Serial.print("forward");

  L_MOTOR->run(FORWARD);
  R_MOTOR->run(FORWARD);
  for (i=0; i<255; i++) {
    L_MOTOR->setSpeed(i);
    R_MOTOR->setSpeed(i);  
    delay(10);
  }
  delay(1000);
  for (i=255; i!=0; i--) {
    L_MOTOR->setSpeed(i);
    R_MOTOR->setSpeed(i);  
    delay(10);
  }
  
  Serial.print("backward");

  L_MOTOR->run(BACKWARD);
  R_MOTOR->run(BACKWARD);
  for (i=0; i<255; i++) {
    L_MOTOR->setSpeed(i);
    R_MOTOR->setSpeed(i); 
    delay(10);
  }
  delay(1000);
  for (i=255; i!=0; i--) {
    L_MOTOR->setSpeed(i);
    R_MOTOR->setSpeed(i);  
    delay(10);
  }

  Serial.print("left");

  L_MOTOR->run(BACKWARD);
  R_MOTOR->run(FORWARD);
  for (i=0; i<255; i++) {
    L_MOTOR->setSpeed(i);
    R_MOTOR->setSpeed(i); 
    delay(10);
  }
  delay(1000);
  for (i=255; i!=0; i--) {
    L_MOTOR->setSpeed(i);
    R_MOTOR->setSpeed(i);  
    delay(10);
  }

  Serial.print("right");

  L_MOTOR->run(FORWARD);
  R_MOTOR->run(BACKWARD);
  for (i=0; i<255; i++) {
    L_MOTOR->setSpeed(i);
    R_MOTOR->setSpeed(i); 
    delay(10);
  }
  delay(1000);
  for (i=255; i!=0; i--) {
    L_MOTOR->setSpeed(i);
    R_MOTOR->setSpeed(i);  
    delay(10);
  }

  Serial.print("release");
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
  delay(2000);
}
