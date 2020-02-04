#include <IRremote.h>
#include <IRremoteInt.h>

#define TIMER_PWM_PIN 3
#define btn_pin 4

#define IRZERO = 0
#define IRFLAT = 1
#define IRBLNK = 2

int IRmode = 1;
long time = 0;
int btnPress;
int debounce = 200;
int previous = LOW;

IRsend irsend;

void beginSerial()
{
  Serial.begin(9600);
  Serial.println(F("Main Program"));
  Serial.println(F("-----------------------------------------"));
}

void setUpPins(){
  pinMode(TIMER_PWM_PIN, OUTPUT);
  pinMode(btn_pin, INPUT_PULLUP);
}

void setUpIR(){
  digitalWrite(TIMER_PWM_PIN, LOW);

  int khz = 38;
  IRsend myIR;

  myIR.enableIROut(khz);
  Serial.println(F("IR Enabled at 38kHz"));
}

void setEmitterMode(){

  IRmode = (IRmode + 1)%3;
  
  switch (IRmode){
    
    case 0:
      TIMER_DISABLE_PWM;
      break;
      
    case 1:
      TIMER_ENABLE_PWM;
      break;
      
     case 2:
      TIMER_ENABLE_PWM;
      delayMicroseconds(250);
      TIMER_DISABLE_PWM;
      delayMicroseconds(250);
      break;
      
    }
}

void setup() {
  // put your setup code here, to run once:
  beginSerial();
  setUpPins();
  setUpIR();  
}

void loop() {
  // put your main code here, to run repeatedly:
  setEmitterMode();

  btnPress = digitalRead(btn_pin);

  if (btnPress == HIGH && previous == LOW && millis() - time > debounce){
    setEmitterMode();
    time = millis();
  }

  previous = btnPress;
}