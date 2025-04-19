#include <Servo.h>

Servo ESC1; 
Servo ESC2;
int Speed; 

void setup(){
Serial.begin(9600);
ESC1.attach(9,1000,2000);
ESC2.attach(10,1000,2000);
}

void loop(){
  Speed = analogRead(A0);
  Speed = map(Speed, 0, 1023, 0, 180);
  ESC1.write(Speed);
  ESC2.write(Speed);
  Serial.println(Speed);
}
