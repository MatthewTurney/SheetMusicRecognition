// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);  // attaches the servo on pin 9 to the servo object 
} 
 
 
void loop() 
{ 
  for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    analogWrite(10, pos*255/180);              // tell servo to go to position in variable 'pos' 
    digitalWrite(13, HIGH-digitalRead(13)); 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    analogWrite(10, pos*255/180);              // tell servo to go to position in variable 'pos' 
    digitalWrite(13, HIGH-digitalRead(13)); 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
} 
