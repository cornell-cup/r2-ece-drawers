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
