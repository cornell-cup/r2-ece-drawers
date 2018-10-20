/*
Created by Jackie Woo

Controls the drawer motor
As of 10/20/18 the delay (which controls how long 
the motor moves in a certain direction) has not been
optimized for R2's drawers specifically.
*/

#include <stdint.h>
#include <Servo.h>

#define PIN_HEAD 5
Servo drawer;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  drawer.attach(PIN_HEAD);
}

void loop() {
  drawer.write(0);
  delay(1000);
  drawer.write(180);
  delay(1000);
  
}
