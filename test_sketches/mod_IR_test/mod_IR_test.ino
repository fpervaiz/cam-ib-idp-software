/*
 * IRremote: IRsendRawDemo - demonstrates sending IR codes with sendRaw
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 *
 * IRsendRawDemo - added by AnalysIR (via www.AnalysIR.com), 24 August 2015
 *
 * This example shows how to send a RAW signal using the IRremote library.
 * The example signal is actually a 32 bit NEC signal.
 * Remote Control button: LGTV Power On/Off. 
 * Hex Value: 0x20DF10EF, 32 bits
 * 
 * It is more efficient to use the sendNEC function to send NEC signals. 
 * Use of sendRaw here, serves only as an example of using the function.
 * 
 */


#include <IRremote.h>
#include <IRremoteInt.h>

#define TIMER_PWM_PIN 3 //defined in the library.
IRsend irsend;

void setup()
{
 pinMode(TIMER_PWM_PIN, OUTPUT);
 digitalWrite(TIMER_PWM_PIN, LOW);
  int khz = 38; // 38kHz carrier frequency for the NEC protocol
  IRsend myIR;

 
  myIR.enableIROut(khz);
  // TIMER_ENABLE_PWM;
}

void loop() {
  
  
    TIMER_ENABLE_PWM;
    delayMicroseconds(250);
    TIMER_DISABLE_PWM;
    delayMicroseconds(250);
  
  
}
