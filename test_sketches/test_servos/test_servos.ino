#include <Servo.h>

Servo a;
Servo t;

int a_pos = 0;
int t_pos = 0;

int val;

void setup()
{
  Serial.begin(9600);

  a.attach(9);
  t.attach(10);
}

void loop()
{
  if (Serial.available())
  {

    int val = Serial.read();

    if (val == 'd') {
      a_pos += 1;
      a.write(a_pos);
    }

    if (val == 'a') {
      a_pos -= 1;
      a.write(a_pos);
    }

    if (val == 'q') {
      t_pos += 1;
      t.write(t_pos);
    }

    if (val == 'e') {
      t_pos -= 1;
      t.write(t_pos);
    }
    
    Serial.println(a_pos);
    Serial.println(t_pos);

    delay(15);    

  }
}