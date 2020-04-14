/*  This node is developed to read sensor values from 
 *  8 different sensors. It is developed for an autonoumus
 *  differential drive robot where 4 sensor are in the front
 *  and 4 in the back. Depending on the velocity, different
 *  sensor are triggered and read from 
 *  Developed by: 
 *  Daniel Söderberg and Martin Gordon 
 */

/* Import ros, messages and sensor library*/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <USSensor.h>
#include <USSensor.h>

/* Declare nodehandle for the node*/
ros::NodeHandle nh;

/*Declare global velocity, different ultrasonic sensors
will be triggered depening on the velocites */
float velocity = 0; 
float angular_velocity = 1; 

/*Array of all sensor pins */
int senPinsFront[] = {22,23,24,25};  
int senPinsBack[] = {42,43,44,45};  

/*Create an object with USSensor(int *pointer, int n), where 
*pointer is a pointer to the first object of the pin-array, 
and n is the total number of pins*/
USSensor front_sensors(26,&senPinsFront[0], sizeof(senPinsFront)/sizeof(senPinsFront[0]));  
USSensor back_sensors(46,&senPinsBack[0], sizeof(senPinsBack)/sizeof(senPinsBack[0]));

/* Callback function for the velocity */
void velocity_cb(const geometry_msgs::Twist& vel)
{
  velocity = vel.linear.x; 
  angular_velocity = vel.angular.z; 
  digitalWrite(7,HIGH); 
  delay(100); 
  digitalWrite(7,LOW); 
  
}

/*Define subscriber for the velocity, publisher for 
ultrasonic sound and range message*/
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &velocity_cb); 
sensor_msgs::Range range_front; 
sensor_msgs::Range range_back; 
ros::Publisher pub_range_front("ultrasound_front", &range_front); 
ros::Publisher pub_range_back("ultrasound_back", &range_back); 


void setup() {
  /*Initiate node, subscribers and publishers*/
  nh.initNode(); 
  nh.subscribe(sub_vel); 
  nh.advertise(pub_range_front); 
  nh.advertise(pub_range_back); 

  /*Initiate the range messages*/
  range_front.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_back.radiation_type = sensor_msgs::Range::ULTRASOUND;
  char frameid[] = "ultrasound";
  range_front.header.frame_id = frameid; 
  range_back.header.frame_id = frameid;
  range_front.field_of_view = 0.1; 
  range_back.field_of_view = 0.1;
  range_front.min_range = 3.0;
  range_front.max_range = 600.47;
  range_back.min_range = 3.0;
  range_back.max_range = 600.47;

  pinMode(7,OUTPUT); 
  digitalWrite(7,HIGH); 
  delay(100); 
  digitalWrite(7,LOW); 
}

/* Mail loop to keep the function going*/ 
void loop() {
  trig_correct_sensors();
  nh.spinOnce(); 
  delay(250); 
}

void trig_correct_sensors()
{
  /*If the robot is drivning forward, check only the front sensors
   * If robot is driving backwards, check only backsensors
   * If robot is turning in place, trig both front and back sensor 
   * If the robot is standing still, do nothing
   */
  if(velocity > 0)
  {
    range_front.range = (float) front_sensors.distance(); 
    pub_range_front.publish(&range_front); 
  }
  else if(velocity < 0)
  {
    range_back.range = (float) back_sensors.distance(); 
    pub_range_back.publish(&range_back); 
  }
  else if(velocity == 0)
  {
    range_front.range = (float) front_sensors.distance(); 
    range_back.range = (float) back_sensors.distance(); 
    pub_range_back.publish(&range_back);
    pub_range_front.publish(&range_front); 
    
  }
  
}
