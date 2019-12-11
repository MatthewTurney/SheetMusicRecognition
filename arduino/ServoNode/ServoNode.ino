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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;

int key_vals[] = {120, 110, 100, 90, 80, 70, 60, 50}; //C, D, E, F, G, A, B, C
int down_val = 80;
int up_val = 100;
int bpm = 60;
float move_delay = 0.15;
float up_delay = 0.02;

void robot_cb( const std_msgs::Float32MultiArray& cmd_msg){
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led    
  int i = 0;
  /*
  for(std::vector<signed char>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
    pub2.publish(up_val)

    for note in music.notes:

        note.rest_before += move_delay + up_delay
        note.duration -= (move_delay + up_delay)

        start_time = time.time()

        rate.sleep()

        while time.time() - up_delay < note.rest_before:
            rate.sleep()

        pub1.publish(key_vals[note.key])
        print("pub1: " + str(key_vals[note.key]))

        rate.sleep()

        while time.time() - start_time < note.rest_before:
            rate.sleep()

        pub2.publish(down_val)
        #print("pub2: down")

        start_time = time.time()

        rate.sleep()

        while time.time() - start_time < note.duration * 240 / bpm:
            rate.sleep()

        pub2.publish(up_val)
  }
  */  
}

ros::Subscriber<std_msgs::Float32MultiArray> robot("robot", robot_cb);

void setup(){
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  nh.initNode();
  nh.subscribe(robot);
  
  servo1.attach(9); //attach it to pin 9
  servo1.write(key_vals[sizeof(key_vals)/2]);

  servo2.attach(10);
  servo2.write(up_val);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
