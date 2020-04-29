  #include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <stdlib.h>
#include <Servo.h>

//Declare node handle and timer for ultrasonic sensor
ros::NodeHandle nh; 
long range_time; 
//Declare servomotors
Servo servoLeft; 
Servo servoRight; 


void velocityCallback(const geometry_msgs::Twist& vel)
{
  float left_wheel = vel.linear.x - (vel.angular.z)*5.4;
  float right_wheel = vel.linear.x + (vel.angular.z)*5.4;
  int v_L = (int) (1500-(100/15.32)*left_wheel); 
  int v_R = (int) (1500+(100/15.32)*right_wheel);
  servoLeft.writeMicroseconds(v_L); 
  servoRight.writeMicroseconds(v_R); 
  steer_led(vel.linear.x,vel.angular.z); 
}


ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &velocityCallback); 

 
void setup() {
  //Initiate node
  nh.initNode(); 
  
  //Initiate publisher and subsriber
  nh.subscribe(cmd_sub);
  

  //Intiate servos
  servoLeft.attach(13); 
  servoRight.attach(12); 
  servoLeft.writeMicroseconds(1500); 
  servoRight.writeMicroseconds(1500); 

  //Initiate LED's
  pinMode(8,OUTPUT); 
  pinMode(7,OUTPUT); 
  steer_led(0,0); 
}

void loop() {

  //Kepp node from shutting down
  nh.spinOnce(); 
}



void steer_led(float lin, float ang)
{
  if(lin == 0 && ang == 0)
  {
    for(int i = 0; i<3; i++)
    {
      digitalWrite(8,HIGH); 
      digitalWrite(7,HIGH);
      delay(50); 
      digitalWrite(8,LOW); 
      digitalWrite(7,LOW);
      delay(50);
    }
  }
  else if(ang > 1.0)
  {
     digitalWrite(8,LOW); 
     digitalWrite(7,HIGH); 
  }
  else if(ang < -1.0)
  {
     digitalWrite(8,HIGH); 
     digitalWrite(7,LOW); 
  }
  else
  {
     digitalWrite(8,HIGH); 
     digitalWrite(7,HIGH);
  }
  
}
