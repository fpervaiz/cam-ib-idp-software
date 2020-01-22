#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// DEFINITIONS

#define pin_sens_ultrasound_trigger 10
#define pin_sens_ultrasound_echo 13

#define pin_indicator_move_led 2

#define port_motor_left 1
#define port_motor_right 2

#define const_move_indicator_period 250

// VARIABLES

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(port_motor_left);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(port_motor_right);

unsigned long prev_move_indicator_millis = 0;

int cmd_move = 0; // 0 = stop, 1 = forward; 2 = backward; 3 = pivot left; 4 = pivot right; 5 = rotate left; 6 = rotate right
long time = 0;
long debounce = 200;
int state = HIGH;
int hitWall;
int previousHitWall = LOW;


// FUNCTIONS

void beginSerial()
{
  Serial.begin(115200);
  Serial.println(F("Main Program"));
  Serial.println(F("-----------------------------------------"));
}

void setupUltrasound()
{
  pinMode(pin_sens_ultrasound_trigger, OUTPUT);
  pinMode(pin_sens_ultrasound_echo, INPUT);
}

void setupDriveMotors()
{
  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
}

void setupIndicators()
{
  pinMode(pin_indicator_move_led, OUTPUT);
}

bool is_moving()
{
  return !(cmd_move == = 0);
}

void toggleMoveIndicator()
{
  if (is_moving())
  {
    digitalWrite(pin_indicator_move_led, !digitalRead(pin_indicator_move_led));
  }
  else
  {
    digitalWrite(pin_indicator_move_led, 0);
  }
}

void driveMotors()
{
  switch (cmd_move)
  {
  case 0: // Stop
    L_MOTOR->setSpeed(0);
    R_MOTOR->setSpeed(0);
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(RELEASE);
    break;
  case 1: // Forward

    break;
  case 2: // Backward
    break;
  case 3: // Pivot left
    break;
  case 4: // Pivot right
    break;
  case 5: // Rotate left
    break;
  case 6: // Rotate right
    break;
  }
}

// PROGRAM

void setup()
{
  beginSerial();
  setupUltrasound();
  setupDriveMotors();
}

void loop()
{
  unsigned long current_millis = millis();

  if (current_millis - prev_move_indicator_millis >= const_move_indicator_period)
  {
    toggleMoveIndicator();
  }

  driveMotors();
  
  hitWall = digitalRead(microPin);
  
  if (hitWall == HIGH && previous == LOW && millis() - time > debounce){
    hitAWall(); }

  previousHitWall = hitWall;
}
