#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// DEFINITIONS

#define pin_sens_ultrasound_trigger 10
#define pin_sens_ultrasound_echo 13

#define pin_sens_optor_l A0
#define pin_sens_optor_c A1
#define pin_sens_optor_r A2

#define pin_indicator_move_led 2

#define pin_btn_start 3

#define port_motor_left 1
#define port_motor_right 2

#define const_motor_full_speed 255
#define const_motor_half_speed 128

#define const_move_indicator_period 250

#define const_sens_optor_l_threshold 500
#define const_sens_optor_c_threshold 250
#define const_sens_optor_r_threshold 350
#define const_sens_optor_working_minimum 10
#define const_sens_optor_working_maximum 1000

// Movement command definitions

#define STOP 0
#define FWRD 1
#define RVRS 2
#define PVTL 3
#define PVTR 4
#define ROTL 5
#define ROTR 6

// VARIABLES

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(port_motor_left);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(port_motor_right);

unsigned long prev_move_indicator_millis = 0;

int cmd_move = 0; // 0 = stop, 1 = forward; 2 = backward; 3 = pivot left; 4 = pivot right; 5 = rotate left; 6 = rotate right
int cmd_move_prev = cmd_move;
int cmd_speed = 255;

// FUNCTIONS

// Setup

void beginSerial()
{
  Serial.begin(9600);
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

  // Test each optoreflector
  int r1 = analogRead(pin_sens_optor_l);
  int r2 = analogRead(pin_sens_optor_c);
  int r3 = analogRead(pin_sens_optor_r);
  bool failure = false;

  if (r1 > const_sens_optor_working_maximum || r1 < const_sens_optor_working_minimum)
  {
    // Left optoreflector issue
    failure = true;
  }
  if (r2 > const_sens_optor_working_maximum || r2 < const_sens_optor_working_minimum)
  {
    // Centre optoreflector issue
    failure = true;
  }
  if (r3 > const_sens_optor_working_maximum || r3 < const_sens_optor_working_minimum)
  {
    // Right optoreflector issue
    failure = true;
  }

  if (failure)
  {
    Serial.println("Optoreflector readings outside working range!");
    Serial.print(r1);
    Serial.print(" ");
    Serial.print(r2);
    Serial.print(" ");
    Serial.println(r3);

    Serial.println("EXECUTION HALTED");
    while (true)
    {
      delay(10000);
    }
  }
}

void setupBtns()
{
  pinMode(pin_btn_start, INPUT_PULLUP);
}

// Logic

bool startBtnPressed()
{
  return !digitalRead(pin_btn_start);
}

bool isMoving()
{
  return !(cmd_move == 0);
}

void toggleMoveIndicator()
{
  if (isMoving())
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
  if (cmd_move != cmd_move_prev)
  {
    cmd_move_prev = cmd_move;
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
}

bool optoIsDark(int opto_pin, int threshold)
{
  // Serial.println(analogRead(opto_pin));
  return analogRead(opto_pin) < threshold;

}

void lineFollow()
{
  bool b1 = optoIsDark(pin_sens_optor_l, const_sens_optor_l_threshold);
  bool b2 = optoIsDark(pin_sens_optor_c, const_sens_optor_c_threshold);
  bool b3 = optoIsDark(pin_sens_optor_r, const_sens_optor_r_threshold);

  if (b1 && b2 && b3)
  {
    // B B B - lost
    cmd_move = STOP;
    Serial.println("Lost.");
  }
  else if (b1 && b2 && !b3)
  {
    // B B W - pivot hard right
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTR;
  }
  else if (b1 && b2 && !b3)
  {
    // B W B - straight - on line
    cmd_speed = const_motor_full_speed;
    cmd_move = FWRD;
  }
  else if (b1 && b2 && !b3)
  {
    // B W W - pivot right
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTR;
  }
  else if (b1 && b2 && !b3)
  {
    // W B B - pivot hard left
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTL;
  }
  else if (b1 && b2 && !b3)
  {
    // W B W - junction - decision - to be implemented
    cmd_speed = const_motor_half_speed;
    cmd_move = FWRD;
  }
  else if (b1 && b2 && !b3)
  {
    // W W B - pivot left
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTL;
  }
  else if (b1 && b2 && !b3)
  {
    // W W W - junction - decision
    cmd_speed = const_motor_full_speed;
    cmd_move = STOP;
  }
  else
  {
    // Undefined reading set
    Serial.println("Undefined optoreflector state:");
    Serial.print(b1);
    Serial.print(" ");
    Serial.print(b2);
    Serial.print(" ");
    Serial.println(b3);
  }
}

void simpleLineFollow()
{
  if (optoIsDark(pin_sens_optor_l, const_sens_optor_l_threshold))
  {
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTL;
  }
  else
  {
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTR;
  }
}

// New vars

#include <Servo.h>

double sig_max, an;
unsigned long tS, tM, curr_time;
int time_rotate = 500;

Servo s_arm;
Servo s_tray;

void loadServo(){
 s_tray.write(160); // Lower tray, assuming arm is already open
 s_arm.write(20);   // Close arm
 s_tray.write(20);  // Raise tray
  
}

void radialSearch(){
  cmd_move = 5;
  sig_max = 0;

  driveMotor();
  
  for (int t = 0; t < time_rotate; t++){
    an = analogRead(IRPin);
    curr_time = millis();
    
    if (an > sig_max){
      sig_max = an; 
      tM = curr_time}

    // delay(2)
    }

  cmd_move = 6;
  curr_time = millis();
  driveMotor();
  delay(curr_time - tM);
  cmd_move = 0;
  driveMotor();
}

// PROGRAM

void setup()
{
  beginSerial();
  setupBtns();
  setupIndicators();
  setupUltrasound();
  setupDriveMotors();

  // Wait till button pressed to start
  Serial.println("Waiting for start button push...");
  while (!startBtnPressed())
  {
    delay(100);
  }

  Serial.println("Starting main loop");
}

void loop()
{
  unsigned long current_millis = millis();

  if (current_millis - prev_move_indicator_millis >= const_move_indicator_period)
  {
    toggleMoveIndicator();
  }

  //simpleLineFollow();
  lineFollow();
  driveMotors();
  delay(10);
}
