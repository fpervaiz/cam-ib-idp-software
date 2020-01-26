#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// DEFINITIONS

#define pin_sens_ultrasound_trigger 10
#define pin_sens_ultrasound_echo 13

// To be replaced with comparators and digital in
#define pin_sens_optor_l A0
#define pin_sens_optor_c A1
#define pin_sens_optor_r A2

#define pin_indicator_move_led 2

#define port_motor_left 1
#define port_motor_right 2

#define const_move_indicator_period 250
#define const_sens_optor_threshold 500

// VARIABLES

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(port_motor_left);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(port_motor_right);

unsigned long prev_move_indicator_millis = 0;

int cmd_move = 0; // 0 = stop, 1 = forward; 2 = backward; 3 = pivot left; 4 = pivot right; 5 = rotate left; 6 = rotate right
int cmd_speed = 255;

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
  AFMS.begin();
  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
}

void setupIndicators()
{
  pinMode(pin_indicator_move_led, OUTPUT);
}

void setupOptors()
{
  pinMode(pin_sens_optor_l, INPUT);
  pinMode(pin_sens_optor_c, INPUT);
  pinMode(pin_sens_optor_r, INPUT);
}

bool is_moving()
{
  return !(cmd_move == 0);
}

void toggleMoveIndicator()
{
  if (is_moving())
  {
    digitalWrite(pin_indicator_move_led, (PinStatus)!digitalRead(pin_indicator_move_led));
    prev_move_indicator_millis = millis();
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
    L_MOTOR->run(FORWARD);
    R_MOTOR->run(FORWARD);
    L_MOTOR->setSpeed(cmd_speed);
    R_MOTOR->setSpeed(cmd_speed);
    break;
  case 2: // Backward
    L_MOTOR->run(BACKWARD);
    R_MOTOR->run(BACKWARD);
    L_MOTOR->setSpeed(cmd_speed);
    R_MOTOR->setSpeed(cmd_speed);
    break;
  case 3: // Pivot left
    Serial.println("case 3");
    L_MOTOR->run(FORWARD);
    R_MOTOR->run(RELEASE);
    L_MOTOR->setSpeed(cmd_speed);
    R_MOTOR->setSpeed(cmd_speed);
    break;
  case 4: // Pivot right
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(FORWARD);
    L_MOTOR->setSpeed(cmd_speed);
    R_MOTOR->setSpeed(cmd_speed);
    break;
  case 5: // Rotate left
    L_MOTOR->run(FORWARD);
    R_MOTOR->run(BACKWARD);
    L_MOTOR->setSpeed(cmd_speed);
    R_MOTOR->setSpeed(cmd_speed);
    break;
  case 6: // Rotate right
    L_MOTOR->run(BACKWARD);
    R_MOTOR->run(FORWARD);
    L_MOTOR->setSpeed(cmd_speed);
    R_MOTOR->setSpeed(cmd_speed);
    break;
  }
}

bool optoIsDark(int opto_pin)
{
  Serial.println(analogRead(opto_pin));
  return analogRead(opto_pin) < const_sens_optor_threshold;
}

/*void lineFollow()
{
  if (optoIsDark(pin_sens_optor_l) && !optoIsDark(pin_sens_optor_l) && optoIsDark(pin_sens_optor_l))
  {
    // On line - continue straight
    cmd_move = 1;
  }
  else if ()
  {
    // Move left
  }
  else if ()
  {
    // Move right
  }
  else if ()
  {
    // Stop
  }
  
  
}*/

void simpleLineFollow() {
  if (optoIsDark(pin_sens_optor_l)) {
    cmd_move = 4;
  }
  else {
    cmd_move = 3;
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

  simpleLineFollow();
  Serial.println(cmd_move);
  driveMotors();
  delay(100);
}
