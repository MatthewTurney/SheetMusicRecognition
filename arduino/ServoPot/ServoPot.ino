/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */
//96 = 0
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;
std_msgs::UInt16 bpm;
int lowest = 30;
int highest = 120;

void servo1_cb( const std_msgs::UInt16& cmd_msg){
  servo1.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void servo2_cb( const std_msgs::UInt16& cmd_msg){
  servo2.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::UInt16> sub1("servo1", servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("servo2", servo2_cb);
ros::Publisher pot("pot", &bpm);

int old_val = 0;
int pot_val = 1;

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(pot);
  
  servo1.attach(9); //attach it to pin 9
  servo1.write(90);

  servo2.attach(10);
  servo2.write(90);
  pinMode(A0, INPUT);
}

void loop(){
  pot_val = analogRead(A0) / 1024.0 * (highest - lowest) + lowest;
  if(pot_val != old_val){
    bpm.data = pot_val;
    pot.publish(&bpm);
    old_val = pot_val;
  }
  nh.spinOnce();
  delay(1);
}
