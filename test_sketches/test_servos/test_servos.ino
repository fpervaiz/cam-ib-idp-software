#include <Servo.h>

Servo s_arm;
Servo s_tray;

int pos_arm = 0;
int pos_tray = 0;

void setup() {
  Serial.begin(115200);
  s_arm.attach(9);
  s_tray.attach(10);
  s_arm.write(20);
  s_tray.write(20);
}

void loop() {
  /*for (int pos = 20; pos <= 160; pos += 1) {
    s_arm.write(pos);
    s_tray.write(pos);
    Serial.println(pos);
    delay(15);
    
  }
  for (int pos = 160; pos >= 20; pos -= 1) {
    s_arm.write(pos);
    s_tray.write(pos);
    Serial.println(pos);
    delay(15);
  }*/
  /*s_arm.write(20);
  s_tray.write(20);
  delay(2000);
  s_arm.write(180);
  s_tray.write(180);
  delay(2000);*/
}
