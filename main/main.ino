#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PubSubClient.h>
#include <WiFiNINA.h>
#include <Servo.h>

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

#define port_servo_arm 9
#define port_servo_tray 10

#define const_move_indicator_period 250

#define const_sens_optor_l_threshold 300 // 500
#define const_sens_optor_c_threshold 400 // 250
#define const_sens_optor_r_threshold 350 // 350
#define const_sens_optor_working_minimum 5
#define const_sens_optor_working_maximum 1023

#define const_servo_pos_tray_up 135
#define const_servo_pos_tray_down 145
#define const_servo_pos_arm_in 110
#define const_servo_pos_arm_out 25

#define const_topic_bot_cmd_stage "/idp/bot/cmd_stage"
#define const_topic_bot_stt_stage "/idp/bot/stt_stage"
#define const_topic_bot_stt_drop_stage "/idp/bot/stt_drop_stage"
#define const_topic_bot_debug "/idp/bot/debug"
#define const_topic_bot_cmd_move "/idp/bot/cmd_move"
#define const_topic_bot_cmd_speed "/idp/bot/cmd_speed"
#define const_topic_bot_cmd_mech "/idp/bot/cmd_mech"

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

Servo servo_arm;
Servo servo_tray;

unsigned long prev_move_indicator_millis = 0;

unsigned long temp_timestore = 0;
int temp_time_interval;
bool temp_wait_complete = true;

int cmd_move = STOP;
int cmd_move_prev = cmd_move;

int cmd_speed = 255;
int cmd_speed_prev = cmd_speed;

bool line_follow_complete = false;
bool mech_open_cmd_recvd = false;
bool mech_ready_to_pickup = false;
bool mech_open_triggered = false;

int task_state = 0;

char ssid[] = "IDP_L101";
char pass[] = ">r063W83";
int status = WL_IDLE_STATUS;
IPAddress mqtt_server(192, 168, 137, 1);

WiFiClient net;
PubSubClient mqc(net);

// FUNCTIONS

// Comms setup

void beginSerial()
{
  Serial.begin(9600);
  Serial.println(F("-----------------------------------------"));
  Serial.println(" ");
  Serial.println(F("Main Program"));
  Serial.println(F("-----------------------------------------"));
}

void setupWifi()
{
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE)
    {
        Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true)
            ;
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION)
    {
        Serial.println("Please upgrade the firmware");
    }

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to WPA SSID: ");
        Serial.println(ssid);
        // Connect to WPA/WPA2 network:
        status = WiFi.begin(ssid, pass);

        // wait 3 seconds for connection:
        delay(3000);
    }

    // you're connected now, so print out the data:
    Serial.println("Connected to WiFi.");
    //printCurrentNet();
    //printWifiData();
}

// MQTT functions

void onMessageReceived(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);

    Serial.print("] ");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();

    payload[length] = '\0';
    int payload_val = atoi((char *)payload);

    String str_topic = String((char*)topic);
    int str_topic_len = str_topic.length() + 1; 
    char buf_topic[str_topic_len];
    str_topic.toCharArray(buf_topic, str_topic_len);

    mqc.publish(const_topic_bot_debug, "Recieved message");
    mqc.publish(const_topic_bot_debug, buf_topic);

    for (int i = 0; i < length; i++)
    {
      Serial.print((char)payload[i]);
      //mqc.publish(const_topic_bot_debug, String((char)payload[i]).c_str());
    }

    Serial.println();
    //mqc.publish(const_topic_bot_debug, " ");

    if (strcmp(buf_topic, const_topic_bot_cmd_move) == 0) {
      cmd_move = payload_val;

      Serial.print("Set cmd_move to ");
      Serial.print(cmd_move);
      Serial.print(" - received ");
      Serial.println(payload_val);

      mqc.publish(const_topic_bot_debug, "Set command move to:");
      mqc.publish(const_topic_bot_debug, String(cmd_move).c_str());
    }
    else if (strcmp(buf_topic, const_topic_bot_cmd_speed) == 0) {
      cmd_speed = payload_val;

      Serial.print("Set cmd_speed to ");
      Serial.print(cmd_speed);
      Serial.print(" - received ");
      Serial.println(payload_val);

      mqc.publish(const_topic_bot_debug, "Set command speed to:");
      mqc.publish(const_topic_bot_debug, String(cmd_speed).c_str());
    }
    else if (strcmp(buf_topic, const_topic_bot_cmd_stage) == 0) {
      task_state = payload_val;

      Serial.print("Set task_state to ");
      Serial.print(task_state);
      Serial.print(" - received ");
      Serial.println(payload_val);

      mqc.publish(const_topic_bot_debug, "Set task state to:");
      mqc.publish(const_topic_bot_debug, String(task_state).c_str());
    }
    else if (strcmp(buf_topic, const_topic_bot_cmd_mech) == 0) {

      Serial.println("Opening mechanism for loading");
      mech_open_cmd_recvd = true;

    }
    else {
      mqc.publish(const_topic_bot_debug, "No match found for topic");
    }
    
}

void connectMqtt()
{
    while (!mqc.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqc.connect("arduinoClient"))
        {
            Serial.println("connected");
            // Once connected, publish an announcement...
            mqc.publish(const_topic_bot_debug, "MQTT connected.");

            // ... and subscribe
            mqc.subscribe(const_topic_bot_cmd_move);
            mqc.subscribe(const_topic_bot_cmd_speed);
            mqc.subscribe(const_topic_bot_cmd_stage);
            mqc.subscribe(const_topic_bot_cmd_mech);

        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqc.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void setupMqtt()
{
    mqc.setServer(mqtt_server, 1883);
    mqc.setCallback(onMessageReceived);

    connectMqtt();
}

// Sensor setup

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

void setupServos()
{
  servo_arm.write(const_servo_pos_arm_in);
  servo_tray.write(const_servo_pos_tray_up);

  servo_arm.attach(port_servo_arm);
  servo_tray.attach(port_servo_tray);

  delay(500);

  servo_arm.detach();
  servo_tray.detach();
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

    mqc.publish(const_topic_bot_debug, "Optoreflector readings outside working range!");
    mqc.publish(const_topic_bot_debug, String(r1).c_str());
    mqc.publish(const_topic_bot_debug, String(r2).c_str());
    mqc.publish(const_topic_bot_debug, String(r3).c_str());
    
    Serial.println("EXECUTION HALTED");
    mqc.publish(const_topic_bot_debug, "EXECUTION HALTED");

    while (true)
    {
      delay(10000);
    }
  }
  else {
    Serial.println("All optoreflector readings inside range.");
    Serial.print(r1);
    Serial.print(" ");
    Serial.print(r2);
    Serial.print(" ");
    Serial.println(r3);

    mqc.publish(const_topic_bot_debug, "All optoreflector readings inside range.");
    mqc.publish(const_topic_bot_debug, String(r1).c_str());
    mqc.publish(const_topic_bot_debug, String(r2).c_str());
    mqc.publish(const_topic_bot_debug, String(r3).c_str());
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
  if (cmd_move != cmd_move_prev || cmd_speed != cmd_speed_prev)
  {
    cmd_move_prev = cmd_move;
    cmd_speed_prev = cmd_speed;

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
      L_MOTOR->run(BACKWARD);
      R_MOTOR->run(RELEASE);
      L_MOTOR->setSpeed(cmd_speed);
      R_MOTOR->setSpeed(cmd_speed);
      break;
    case 4: // Pivot right
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(BACKWARD);
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
    mqc.publish(const_topic_bot_debug, "Lost.");
  }
  else if (b1 && b2 && !b3)
  {
    // B B W - pivot hard right
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTR;
  }
  else if (b1 && !b2 && b3)
  {
    // B W B - straight - on line
    cmd_speed = const_motor_full_speed;
    cmd_move = FWRD;
  }
  else if (b1 && !b2 && !b3)
  {
    // B W W - pivot right
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTR;
  }
  else if (!b1 && b2 && b3)
  {
    // W B B - pivot hard left
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTL;
  }
  else if (!b1 && b2 && !b3)
  {
    // W B W - junction - decision - to be implemented
    cmd_speed = const_motor_half_speed;
    cmd_move = FWRD;
  }
  else if (!b1 && !b2 && b3)
  {
    // W W B - pivot left
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTL;
  }
  else if (!b1 && !b2 && !b3)
  {
    // W W W - junction - decision
    cmd_speed = const_motor_full_speed;
    cmd_move = STOP;
    line_follow_complete = true;
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

    mqc.publish(const_topic_bot_debug, "Undefined optoreflector state:");
    mqc.publish(const_topic_bot_debug, String(b1).c_str());
    mqc.publish(const_topic_bot_debug, String(b2).c_str());
    mqc.publish(const_topic_bot_debug, String(b3).c_str());
  }
}

void lineFollow2()
{
  bool b1 = optoIsDark(pin_sens_optor_l, const_sens_optor_l_threshold);
  bool b2 = optoIsDark(pin_sens_optor_c, const_sens_optor_c_threshold);
  bool b3 = optoIsDark(pin_sens_optor_r, const_sens_optor_r_threshold);

  if (b1 && b2 && b3)
  {
    // B B B - lost
    cmd_speed = const_motor_full_speed;
    cmd_move = STOP;
    Serial.println("Complete.");
    mqc.publish(const_topic_bot_debug, "lf2 complete.");
    line_follow_complete = true;
  }
  else if (b1 && b2 && !b3)
  {
    // B B W - pivot hard right
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTR;
  }
  else if (b1 && !b2 && b3)
  {
    // B W B - straight - on line
    cmd_speed = const_motor_full_speed;
    cmd_move = FWRD;
  }
  else if (b1 && !b2 && !b3)
  {
    // B W W - pivot right
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTR;
  }
  else if (!b1 && b2 && b3)
  {
    // W B B - pivot hard left
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTL;
  }
  else if (!b1 && b2 && !b3)
  {
    // W B W - junction - decision - to be implemented
    cmd_speed = const_motor_half_speed;
    cmd_move = FWRD;
  }
  else if (!b1 && !b2 && b3)
  {
    // W W B - pivot left
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTL;
  }
  else if (!b1 && !b2 && !b3)
  {
    // W W W - junction - decision
    cmd_speed = const_motor_half_speed;
    cmd_move = FWRD;
    //line_follow_complete = true;
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

    mqc.publish(const_topic_bot_debug, "Undefined optoreflector state:");
    mqc.publish(const_topic_bot_debug, String(b1).c_str());
    mqc.publish(const_topic_bot_debug, String(b2).c_str());
    mqc.publish(const_topic_bot_debug, String(b3).c_str());
  }
}

void lineFollow3()
{
  bool b1 = optoIsDark(pin_sens_optor_l, const_sens_optor_l_threshold);
  bool b2 = optoIsDark(pin_sens_optor_c, const_sens_optor_c_threshold);
  bool b3 = optoIsDark(pin_sens_optor_r, const_sens_optor_r_threshold);

  if (b1 && b2 && b3)
  {
    // B B B - lost
    cmd_move = STOP;
    Serial.println("Lost.");
    mqc.publish(const_topic_bot_debug, "Lost.");
  }
  else if (b1 && b2 && !b3)
  {
    // B B W - pivot hard right
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTR;
  }
  else if (b1 && !b2 && b3)
  {
    // B W B - straight - on line
    cmd_speed = const_motor_full_speed;
    cmd_move = FWRD;
  }
  else if (b1 && !b2 && !b3)
  {
    // B W W - pivot right
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTR;
  }
  else if (!b1 && b2 && b3)
  {
    // W B B - pivot hard left
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTL;
  }
  else if (!b1 && b2 && !b3)
  {
    // W B W - junction - decision - rightward bias to exit cave right
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTR;
  }
  else if (!b1 && !b2 && b3)
  {
    // W W B - pivot left
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTL;
  }
  else if (!b1 && !b2 && !b3)
  {
    // W W W - junction - rightward bias
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTR;
    //line_follow_complete = true;
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

    mqc.publish(const_topic_bot_debug, "Undefined optoreflector state:");
    mqc.publish(const_topic_bot_debug, String(b1).c_str());
    mqc.publish(const_topic_bot_debug, String(b2).c_str());
    mqc.publish(const_topic_bot_debug, String(b3).c_str());
  }
}

/*
void simpleLineFollow()
{
  if (optoIsDark(pin_sens_optor_c, const_sens_optor_c_threshold))
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
*/

// PROGRAM

void setup()
{
  // Reset state
  //task_state = -1;

  setupDriveMotors();

  beginSerial();
  setupWifi();
  setupMqtt();

  mqc.publish(const_topic_bot_debug, " ");
  mqc.publish(const_topic_bot_debug, "-----------------------------------");
  mqc.publish(const_topic_bot_debug, "L101 Main Bot Program");
  mqc.publish(const_topic_bot_debug, "Arduino connected.");

  setupServos();  
  setupBtns();
  setupIndicators();
  setupUltrasound();
  setupOptors();

  // Send 0 state to driver
  task_state = 0;
  mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());

  // Wait for start button
  Serial.println("Waiting for start button push...");
  mqc.publish(const_topic_bot_debug, "Waiting for start button push...");
  while (!startBtnPressed())
  {
    delay(100);
    mqc.loop();
  }

  task_state = 1;
  Serial.println("Starting main loop");
  mqc.publish(const_topic_bot_debug, "Starting main loop");  
  mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());

}

void loop()
{

  if (!mqc.connected()) {
    int temp = cmd_move;
    cmd_move = STOP;
    driveMotors();

    connectMqtt();

    // Force driver to update
    mqc.publish(const_topic_bot_stt_drop_stage, String(task_state).c_str());

    cmd_move = temp;
    driveMotors();

  }

  unsigned long current_millis = millis();

  // Solid amber during health detection otherwise blink amber 2Hz while moving
  if (task_state == 5 && !digitalRead(pin_indicator_move_led)) {
    digitalWrite(pin_indicator_move_led, 1);
  }
  else {
    if (current_millis - prev_move_indicator_millis >= const_move_indicator_period)
    {
      toggleMoveIndicator();
    }
  }

  switch (task_state) {
    case 0:
      break;

    case 1:
      // Line following to start box exit
      if (!line_follow_complete) {
        lineFollow();
      }
      else {
        // Reached border: drive forward to clear border
        if (cmd_move == STOP) {
          temp_timestore = millis();
          temp_time_interval = 1500;
          cmd_move = FWRD;
        }
        else {
          // Task state increment
          if (millis() - temp_timestore > temp_time_interval) {
            cmd_move = STOP;

            task_state = 2;
            mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());

            line_follow_complete = false;
          }
        }        
      }
      break;

    case 2:
      // Line following to enter cave
      if (!line_follow_complete) {
        lineFollow2();
      }
      else {
        // Reached cave entry line - move forward to clear
        cmd_move = FWRD;

        if (temp_wait_complete) {
          temp_timestore = millis();
          temp_time_interval = 1500;
          temp_wait_complete = false;
        }
        else {
          if (millis() - temp_timestore > temp_time_interval) {
            // Cleared cave
            cmd_move = STOP;
            temp_wait_complete = true;
        
            task_state = 3;
            mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());

          }
        }
      }
      break;

    case 3:
      // Computer vision vector control (navigate to victims)
      /*
      if (cmd_speed != const_motor_half_speed) {
        cmd_speed = const_motor_half_speed;
        mqc.publish(const_topic_bot_debug, "Set computer vision control speed");
      }
      //collisionAvoidance();
      */      
      break;
    
    case 4:
      // Computer vision vector control (positioning to load)
      //cmd_speed = const_motor_half_speed;

      if (mech_open_cmd_recvd && !mech_ready_to_pickup && !mech_open_triggered) {
        servo_arm.write(const_servo_pos_arm_out);
        servo_tray.write(const_servo_pos_tray_down);
        
        servo_arm.attach(port_servo_arm);
        servo_tray.attach(port_servo_tray);

        mech_open_triggered = true;
      }
      
      if (temp_wait_complete) {
        temp_timestore = millis();
        temp_time_interval = 500;
        temp_wait_complete = false;
      }
      else {
        if (millis() - temp_timestore > temp_time_interval) {
          // Movement complete - detach setupServos

          servo_arm.detach();
          servo_tray.detach();

          mech_ready_to_pickup = true;
          temp_wait_complete = true;

        }
      }

      //collisionAvoidance();
      break;

    case 5:
      // Health detection wait
      cmd_move = STOP;

      if (temp_wait_complete) {
        temp_timestore = millis();
        temp_time_interval = 3000;
        temp_wait_complete = false;
      }
      else {
        if (millis() - temp_timestore > temp_time_interval) {
          // Health detection wait complete
          temp_wait_complete = true;

          task_state = 6;
          mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());
        }
      }
      
      break;

    case 6:
      // Load victim - BLOCKING CASE
      // To be implemented. Currently just waits some time.
      cmd_speed = 48;
      cmd_move = RVRS;
      driveMotors();
      delay(500);
      cmd_move = STOP;
      driveMotors();

      servo_arm.write(const_servo_pos_arm_out);
      servo_arm.attach(port_servo_arm);

      delay(50);

      for (int pos = const_servo_pos_arm_out; pos <= const_servo_pos_arm_in; pos += 10) {
        servo_arm.write(pos);
        delay(100);
      }

      servo_arm.detach();

      servo_tray.write(const_servo_pos_tray_up);      
      servo_tray.attach(port_servo_tray);

      delay(500);

      servo_tray.detach();

      task_state = 7;
      mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());

      /*
      if (temp_wait_complete) {
        temp_timestore = millis();
        temp_time_interval = 5000;
        temp_wait_complete = false;
      }
      else {
        if (millis() - temp_timestore > temp_time_interval) {
          // Pickup wait complete
          temp_wait_complete = true;

          task_state = 7;
          mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());
        }
      }
      */

      break;

    case 7:
      // Computer vision vector control (navigate to cave exit)
      //cmd_speed = const_motor_half_speed;
      //collisionAvoidance(); 
      break;

    case 8:
      // Line follow cave exit to triage box
      if (!line_follow_complete) {
        lineFollow3();
      }
      else {
        // Reached triage box line (should never get here)
        cmd_move = STOP;
        
        task_state = 9;
        mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());

        line_follow_complete = false;
      }
      break;

    case 9:
      // Computer vision vector control (reposition for unloading)
      //cmd_speed = const_motor_half_speed;
      //collisionAvoidance(); 
      break;

    case 10:
      // Unload victim - BLOCKING CASE
      // To be implemented. Currently just waits some time.
      cmd_speed = 192;
      cmd_move = RVRS;
      driveMotors();
      delay(1000);
      cmd_move = STOP;
      driveMotors();

      servo_arm.write(const_servo_pos_arm_out);
      servo_tray.write(const_servo_pos_tray_down);

      servo_arm.attach(port_servo_arm);
      servo_tray.attach(port_servo_tray);

      delay(500);

      cmd_move = FWRD;
      driveMotors();
      delay(1000);
      cmd_move = STOP;
      driveMotors();

      servo_arm.write(const_servo_pos_arm_in);
      servo_tray.write(const_servo_pos_tray_up);

      delay(500);

      servo_arm.detach();
      servo_tray.detach();

      task_state = 11;
      mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());

      /*
      if (temp_wait_complete) {
        temp_timestore = millis();
        temp_time_interval = 5000;
        temp_wait_complete = false;
      }
      else {
        if (millis() - temp_timestore > temp_time_interval) {
          // Victim unload wait complete
          temp_wait_complete = true;

          task_state = 2;
          mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());
        }
      }
      */

      break;

    case 11:
      // Driver decision
      break;

    case 12:
      // Computer vision vector control (returning to start box)
      //collisionAvoidance();
      break;

    case 13:
      // Complete
      // Do whatever
      cmd_move = STOP;
      break;
  }

  /*
  //simpleLineFollow();
  if (!line_follow_complete) {
    lineFollow();
  }
  else {
    // Search
    Serial.println("Line follow complete. Beginning search and rescue.");
    mqc.publish(const_topic_bot_debug, "Line follow complete. Beginning search and rescue.");
    mqc.publish(const_topic_bot_stt_stage, "lf_complete");
  }
  */
  
  driveMotors();

  mqc.loop();
}
