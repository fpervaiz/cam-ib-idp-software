/*
main.ino is the sketch that the on-board Arduino runs, following the typical process of running setup once, then looping loop.
We took advantage of this by running a check of all commands, as well as WiFi connectivity, in the loop command, hence continuously keeping instructions updated.
Originally, when we made use of more sensors, all of their checks were made in the loop command as well. The project report covers how this changed over the four weeks.
The only times this loop is 'broken' is to run the line follower script functions, which make use of their own sensor-command loops.
*/


#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <PubSubClient.h>
#include <WiFiNINA.h>
#include <Servo.h>

// DEFINITIONS

// Names of pins on the Arduino that are made use of. During development, their positions were changed, so using variables was helpful.

#define pin_sens_optor_l A0
#define pin_sens_optor_c A1
#define pin_sens_optor_r A2

#define pin_indicator_move_led 2
#define pin_indicator_mqtt_led 5

#define pin_btn_start 4
#define pin_btn_reset 3

#define port_motor_left 1
#define port_motor_right 2

#define const_motor_full_speed 255
#define const_motor_half_speed 128

#define port_servo_arm 9
#define port_servo_tray 10

#define const_move_indicator_period 250
#define const_transmit_interval 10000

#define const_sens_optor_l_threshold 300 // 500
#define const_sens_optor_c_threshold 400 // 250
#define const_sens_optor_r_threshold 350 // 350
#define const_sens_optor_working_minimum 5
#define const_sens_optor_working_maximum 1023

#define const_servo_pos_tray_up 78
#define const_servo_pos_tray_down 105
#define const_servo_pos_arm_in 145
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
unsigned long prev_transmit_millis = 0;

unsigned long temp_timestore = 0;
int temp_time_interval;
bool temp_wait_complete = true;

int cmd_move = STOP;
int cmd_move_prev = cmd_move;

int cmd_speed = 255;
int cmd_speed_prev = cmd_speed;

float left_correction = 1.1;

bool line_follow_complete = false;
bool mech_open_cmd_recvd = false;
bool mech_ready_to_pickup = false;
bool mech_open_triggered = false;
bool marxism = false;

int task_state = 0;

// Definitions for WiFi communcation, made from the laptop using computer vision, to the Arduino itself

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

void onMqttConnectActions() {
  Serial.println("connected");
  // Once connected, publish an announcement...
  mqc.publish(const_topic_bot_debug, "MQTT connected.");

  // ... and subscribe
  mqc.subscribe(const_topic_bot_cmd_move);
  mqc.subscribe(const_topic_bot_cmd_speed);
  mqc.subscribe(const_topic_bot_cmd_stage);
  mqc.subscribe(const_topic_bot_cmd_mech);
}

void onMessageReceived(char *topic, byte *payload, unsigned int length)
{
    //Serial.print("Message arrived [");
    //Serial.print(topic);

    //Serial.print("] ");
    //for (int i = 0; i < length; i++)
    //{
    //    Serial.print((char)payload[i]);
    //}
    //Serial.println();

    payload[length] = '\0';
    int payload_val = atoi((char *)payload);

    String str_topic = String((char*)topic);
    int str_topic_len = str_topic.length() + 1; 
    char buf_topic[str_topic_len];
    str_topic.toCharArray(buf_topic, str_topic_len);

    //mqc.publish(const_topic_bot_debug, "Recieved message");
    //mqc.publish(const_topic_bot_debug, buf_topic);

    for (int i = 0; i < length; i++)
    {
      Serial.print((char)payload[i]);
      //mqc.publish(const_topic_bot_debug, String((char)payload[i]).c_str());
    }

    Serial.println();
    //mqc.publish(const_topic_bot_debug, " ");

    if (strcmp(buf_topic, const_topic_bot_cmd_move) == 0) {
      cmd_move = payload_val;
      /*
      Serial.print("Set cmd_move to ");
      Serial.print(cmd_move);
      Serial.print(" - received ");
      Serial.println(payload_val);
      */

      mqc.publish(const_topic_bot_debug, "Set command move to:");
      mqc.publish(const_topic_bot_debug, String(cmd_move).c_str());
    }
    else if (strcmp(buf_topic, const_topic_bot_cmd_speed) == 0) {
      cmd_speed = payload_val;
      /*
      Serial.print("Set cmd_speed to ");
      Serial.print(cmd_speed);
      Serial.print(" - received ");
      Serial.println(payload_val);
      */
      mqc.publish(const_topic_bot_debug, "Set command speed to:");
      mqc.publish(const_topic_bot_debug, String(cmd_speed).c_str());
    }
    else if (strcmp(buf_topic, const_topic_bot_cmd_stage) == 0) {
      task_state = payload_val;
      /*
      Serial.print("Set task_state to ");
      Serial.print(task_state);
      Serial.print(" - received ");
      Serial.println(payload_val);
      */
      mqc.publish(const_topic_bot_debug, "Set task state to:");
      mqc.publish(const_topic_bot_debug, String(task_state).c_str());
    }
    else if (strcmp(buf_topic, const_topic_bot_cmd_mech) == 0) {

      //Serial.println("Opening mechanism for loading");
      mqc.publish(const_topic_bot_debug, "Opening mechanism for loading");
      mech_open_cmd_recvd = true;

    }
    else {
      mqc.publish(const_topic_bot_debug, "No match found for topic");
    }

    mqc.publish(const_topic_bot_debug, " ");
    
}

void connectMqtt()
{
    digitalWrite(pin_indicator_mqtt_led, HIGH);
    while (!mqc.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqc.connect("arduinoClient"))
        {
            onMqttConnectActions();
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqc.state());
            Serial.println(" try again in 2 seconds");
            // Wait 2 seconds before retrying
            delay(2000);
        }
    }
    digitalWrite(pin_indicator_mqtt_led, LOW);
}

void setupMqtt()
{
    mqc.setServer(mqtt_server, 1883);
    mqc.setCallback(onMessageReceived);

    connectMqtt();
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
  pinMode(pin_indicator_mqtt_led, OUTPUT);
}

bool setupOptors()
{
  pinMode(pin_sens_optor_l, INPUT);
  pinMode(pin_sens_optor_c, INPUT);
  pinMode(pin_sens_optor_r, INPUT);

  // Test each optoreflector - these sensors have a built in working range, which we have defined as [const_sens_optor_working_minimum, const_sens_optor_working_maximum]
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
    
    /*
    Serial.println("EXECUTION HALTED");
    mqc.publish(const_topic_bot_debug, "EXECUTION HALTED");

    while (true)
    {
      delay(10000);
    }
    */
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

  return failure;
}

void setupBtns()
{
  pinMode(pin_btn_start, INPUT_PULLUP);
  pinMode(pin_btn_reset, INPUT_PULLUP);
}

// Logic. These are short commands that are used in the loop function.

// These two functions check the main two buttons - start and reset. As their scripts are run once to trigger a change of state, no debouncing is needed.
bool startBtnPressed()
{
  return !digitalRead(pin_btn_start);
}

bool resetBtnPressed()
{
  return !digitalRead(pin_btn_reset);
}

bool isMoving()
{
  return !(cmd_move == 0);
}

void toggleMoveIndicator()
{
  if (isMoving())
  {
    // Switches state of the moving indicator light, then logs the time this happens
    digitalWrite(pin_indicator_move_led, (PinStatus)!digitalRead(pin_indicator_move_led));
    prev_move_indicator_millis = millis();
  }
  else
  {
    digitalWrite(pin_indicator_move_led, 0);
  }
}

// This is the main command for motor behaviour. It makes use of the constants defined at the start of the sketch, with global variables
// cmd_move and cmd_speed set each time driveMotors is called.

void driveMotors()
{
  // Only runs the function if the movement type or speed has changed since the last time driveMotors was caused. The new variables are instantly set to the previous ones in this case
  if (cmd_move != cmd_move_prev || cmd_speed != cmd_speed_prev)
  {
    cmd_move_prev = cmd_move;
    cmd_speed_prev = cmd_speed;

    // A switch-case chain is used here to write the correct motor directions and speeds set by the cmd_move and cmd_speed variables.
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
      L_MOTOR->run(RELEASE);
      R_MOTOR->run(FORWARD);
      L_MOTOR->setSpeed(cmd_speed);
      R_MOTOR->setSpeed(cmd_speed);
      break;
    case 4: // Pivot right
      L_MOTOR->run(FORWARD);
      R_MOTOR->run(RELEASE);
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

// Indicates whether an optic sensor is hovering over the black background of the whole board, or the off-white of the tape the robot must follow. This is used in the line follow commands defined below
// If the analogeRead of the optic sensor is below a threshold, optoIsDark returns true. These thresholds were found experimentally for each of the three optic sensors mounted, suggesting variable sensor strength.
bool optoIsDark(int opto_pin, int threshold)
{
  // Serial.println(analogRead(opto_pin));
  return analogRead(opto_pin) < threshold;

}

// The main line follower script
void lineFollow()
{

  // Checks which of the thres sensors (left, right, centre) are dark, suggesting not hovering over a guiding line
  bool b1 = optoIsDark(pin_sens_optor_l, const_sens_optor_l_threshold);
  bool b2 = optoIsDark(pin_sens_optor_c, const_sens_optor_c_threshold);
  bool b3 = optoIsDark(pin_sens_optor_r, const_sens_optor_r_threshold);

  // If all sensors are dark, the robot stops as it is 'lost', it cannot find the line.
  if (b1 && b2 && b3)
  {
    // B B B - lost
    cmd_move = STOP;
    Serial.println("Lost.");
    mqc.publish(const_topic_bot_debug, "Lost.");
    delay(1000);
  }

  // If just the right-hand sensor is light, this suggests the robot is veering to the lef,t so pivots hard right to stay on the line
  else if (b1 && b2 && !b3)
  {
    // B B W - pivot hard right
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTR;
  }

  // If just the central sensor is light, this suggests the robot is straight on the line, so cmd_move is set to FWRD
  else if (b1 && !b2 && b3)
  {
    // B W B - straight - on line
    cmd_speed = const_motor_full_speed;
    cmd_move = FWRD;
  }

  // Similar to the second case, a slower pivot to the right is made if only the left-hand sensor is dark
  else if (b1 && !b2 && !b3)
  {
    // B W W - pivot right
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTR;
  }

  // Hard pivot to the left, as before
  else if (!b1 && b2 && b3)
  {
    // W B B - pivot hard left
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTL;
  }

  // If only the middle sensor is dark, and the outer two are on a line, then the robot is at a junction. The robot is now commanded to move forwar slowly until it reunites with a solid line.
  else if (!b1 && b2 && !b3)
  {
    // W B W - junction - decision - to be implemented
    cmd_speed = const_motor_half_speed;
    cmd_move = FWRD;
  }

  // Pivot to the left, as before
  else if (!b1 && !b2 && b3)
  {
    // W W B - pivot left
    cmd_speed = const_motor_half_speed;
    cmd_move = PVTL;
  }

  // If all the sensors read light, then the robot has hit the horizontal line that marks the end of the guided path. This causes it to stop and finish line following
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
    
    /*if (marxism) {
      cmd_move = PVTL;
    }
    else {
      cmd_move = FWRD;
    }
    */
   cmd_move = FWRD;
  }
  else if (!b1 && !b2 && b3)
  {
    // W W B - pivot left
    cmd_speed = const_motor_full_speed;//half
    cmd_move = PVTL;
  }
  else if (!b1 && !b2 && !b3)
  {
    // W W W - junction - decision - left bias?
    /*
    if (marxism) {
      cmd_speed = const_motor_half_speed;
      cmd_move = PVTL;
    }
    else {
      cmd_speed = const_motor_half_speed;
      cmd_move = FWRD;
    }
    */
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
    delay(1000);
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
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTL;
  }
  else if (!b1 && !b2 && !b3)
  {
    // W W W - junction - decision - rightward bias to exit cave
    cmd_speed = const_motor_full_speed;
    cmd_move = PVTR;
    driveMotors();
    delay(2500);
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

// The setup function is run once, starting with all the previously defined setup functions.

void setup()
{
  // Reset state
  //task_state = -1;

  // Modular setup
  setupDriveMotors();
  beginSerial();
  setupWifi();
  setupMqtt();

  // Publish a confirmation of MQTT setup
  mqc.publish(const_topic_bot_debug, " ");
  mqc.publish(const_topic_bot_debug, "-----------------------------------");
  mqc.publish(const_topic_bot_debug, "L101 Main Bot Program");
  mqc.publish(const_topic_bot_debug, "Arduino connected.");

  setupServos();  
  setupBtns();
  setupIndicators();
  if (setupOptors()) {
    delay(1000);
  }

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

  mqc.loop();

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

  // Logs current time, which is used in various parts of the loop, as well as in other functions.
  unsigned long current_millis = millis();

  // Solid amber during health detection otherwise blink amber 2Hz while moving. task_state 5 indicates health detection
  if (task_state == 5 && !digitalRead(pin_indicator_move_led)) {
    digitalWrite(pin_indicator_move_led, 1);
  }
  // If not detecting health (i.e. moving)
  else {

    // Calls toggleMoveIndicator, which changes the state of the moving light given enough time has passed
    if (current_millis - prev_move_indicator_millis >= const_move_indicator_period)
    {
      toggleMoveIndicator();
    }
  }

  // Transmit state periodically
  /*
  if (current_millis - prev_transmit_millis >= const_transmit_interval)
  {
    prev_transmit_millis = current_millis;
    mqc.publish(const_topic_bot_stt_drop_stage, String(task_state).c_str());
  
  }
  */

  // Check reset button
  if (resetBtnPressed()) {
    task_state = 0;
    cmd_move = STOP;
    driveMotors();
    Serial.println("Soft reset...");
    mqc.publish(const_topic_bot_debug, "Soft reset...");
    delay(1000);
    mqc.publish(const_topic_bot_stt_stage, task_state);
  }

  // This switch-case chain receives the task_state variable, which determines the overall behaviour of the robot. These are split into the modular tasks the robot must complete:
  switch (task_state) {

    case 0: // Starting up, disables the robot for the whole loop function until the start button is pressed. Again, not debouncing is required here - as soon as a high is detected from the start button, the loop is broken and the robot behaviour starts.
      if (startBtnPressed())
      {
        task_state = 1;
        Serial.println("Starting main loop");
        mqc.publish(const_topic_bot_debug, "Starting main loop");  
        mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());
      }          
      break;

    case 1:
      // Line following to start box exit (white box to cave)
      if (!line_follow_complete) {
        // The lineFollow function only changes command once, so it is called on a loop until line_follow_complete is set to true, then the loop is broken.
        lineFollow();
      }
      else {
        // Reached border: drive forward to clear border, entering cave fully
        if (cmd_move == STOP) {
          temp_timestore = millis();
          temp_time_interval = 1500;
          cmd_move = FWRD;
        }
        else {
          // Task state increment, moving the robot behviour directory on to the next task
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
      // Reset
      if (mech_open_triggered || mech_ready_to_pickup || mech_open_cmd_recvd) {
        mech_open_triggered = false;
        mech_ready_to_pickup = false;
        mech_open_cmd_recvd = false;
      }      

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
        mech_ready_to_pickup = true;
      }
      /*
      if (temp_wait_complete) {
        temp_timestore = millis();
        temp_time_interval = 1500;
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
      */

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

      mqc.disconnect();

      cmd_speed = 48;
      cmd_move = RVRS;
      driveMotors();
      delay(1000);
      cmd_move = STOP;
      driveMotors();

      // Turn slight clockwise to collect victim
      cmd_speed = 48;
      cmd_move = ROTL;
      driveMotors();
      delay(1000);
      cmd_move = STOP;
      driveMotors();

      delay(2000);

      //servo_arm.write(const_servo_pos_arm_out);
      //servo_arm.attach(port_servo_arm);
      //delay(1000);

      servo_arm.write(const_servo_pos_arm_in);

      delay(2000);

      /*
      for (int pos = const_servo_pos_arm_out; pos <= const_servo_pos_arm_in; pos += 10) {
        servo_arm.write(pos);
        delay(100);
      }
      */

      servo_arm.detach();

      servo_tray.write(const_servo_pos_tray_up);      
      //servo_tray.attach(port_servo_tray);

      delay(2000);

      servo_tray.detach();
      /*
      if (!mqc.connected()) {
        connectMqtt();

        // Force driver to update
        mqc.publish(const_topic_bot_stt_drop_stage, String(task_state).c_str());
      }
      */

      connectMqtt();
      mqc.loop();
      delay(500);

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
      // Computer vision vector control (positioning for line following cave exit)
      //cmd_speed = const_motor_half_speed;
      //collisionAvoidance();
      line_follow_complete = false; 
      break;

    case 9:
      // Line follow cave exit to triage box
      if (!line_follow_complete) {
        lineFollow3();
      }
      else {
        // Reached triage box line (should never get here)
        cmd_move = STOP;
        
        task_state = 10;
        mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());

        line_follow_complete = false;
      }
      break;

    case 10:
      // Computer vision vector control (reposition for unloading)
      //cmd_speed = const_motor_half_speed;
      //collisionAvoidance(); 
      break;

    case 11:
      // Unload victim - BLOCKING CASE
      // To be implemented. Currently just waits some time.

      mqc.disconnect();

      // Reverse
      cmd_speed = 192;
      cmd_move = RVRS;
      driveMotors();
      delay(3000);
      cmd_move = STOP;
      driveMotors();

      delay(500);

      // Servos

      servo_arm.write(const_servo_pos_arm_out);
      servo_tray.write(const_servo_pos_tray_down);

      servo_arm.attach(port_servo_arm);
      servo_tray.attach(port_servo_tray);

      delay(1000);

      cmd_move = FWRD;
      driveMotors();
      delay(2000);
      cmd_move = STOP;
      driveMotors();

      servo_arm.write(const_servo_pos_arm_in);
      servo_tray.write(const_servo_pos_tray_up);

      delay(1000);

      servo_arm.detach();
      servo_tray.detach();
      /*
      if (!mqc.connected()) {
        connectMqtt();

        // Force driver to update
        mqc.publish(const_topic_bot_stt_drop_stage, String(task_state).c_str());
      }
      */

      connectMqtt();
      mqc.loop();
      delay(500);     

      task_state = 12;
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

    case 12:
      // Driver decision
      line_follow_complete = false;
      temp_wait_complete = true;
      break;

    case 13:
      // Computer vision vector control (returning to start box)
      //collisionAvoidance();
      temp_wait_complete = true;
      break;

    case 14:
      // Complete

      cmd_move = FWRD;
      cmd_speed = const_motor_full_speed;
      
      if (temp_wait_complete) {
        temp_timestore = millis();
        temp_time_interval = 2000;
        temp_wait_complete = false;
      }
      else {
        if (millis() - temp_timestore > temp_time_interval) {
          // Health detection wait complete
          temp_wait_complete = true;
          cmd_move = 0;
          task_state = 0;
          mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());
        }
      }

      break;

    case 15:
      // Line search
      /*
      if (left_wait_complete && right_wait_complete) {
        temp_timestore = millis();
        temp_time_interval = 3000;
        left_wait_complete = false;
      }
      else {
        if (millis() - temp_timestore > temp_time_interval) {
          left_wait_complete = true;
          right_wait_complete = true;
        }
      }
      else {
      if (optoIsDark(pin_sens_optor_c, const_sens_optor_c_threshold)) {
        cmd_move = ROTR;
        cmd_speed = 192;
      }
      else {
        cmd_move = STOP;

        task_state = 2;
        mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());
      }
      */

      // BLOCKING (replaced by CV)
      /*
      cmd_move = ROTL;
      cmd_speed = 192;
      driveMotors();
      delay(1000);
      
      while (optoIsDark(pin_sens_optor_c, const_sens_optor_c_threshold)) {
        cmd_move = ROTL;
        cmd_speed = 72;
        driveMotors();
        mqc.loop();
        delay(10);
      }

      // Line found
      task_state = 2;
      marxism = true;
      mqc.publish(const_topic_bot_stt_stage, String(task_state).c_str());
      */
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
  delay(10);
}
