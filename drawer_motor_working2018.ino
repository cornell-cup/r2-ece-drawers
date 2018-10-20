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
  //pinMode(PIN_HEAD, OUTPUT);
  //drawer.write(90);
  Serial.println("init");
  

}

void loop() {
  Serial.println("Hello");
  drawer.writeMicroseconds(1000); // moves counter clockwise
  Serial.println("after write 1000");
  delay(1500);
  drawer.writeMicroseconds(2000); // moves clockwise
  Serial.println("after write 2000");
  delay(1500);
  
}
