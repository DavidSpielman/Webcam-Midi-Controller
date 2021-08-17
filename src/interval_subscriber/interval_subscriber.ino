#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

void interval_msg(const std_msgs::Int16& msg){
  if(msg.data == 0) {
    Serial.println("No interval selected");   
    }
  else if(msg.data == 1) {
    Serial.println("Interval 1 detected");   
    }
  else if (msg.data == 2) {
    Serial.println("Interval 2 detected");   
  }
  else if (msg.data == 3) {
    Serial.println("Interval 3 detected");   
  }
  else if (msg.data == 4) {
    Serial.println("Interval 4 detected");   
  }
  else if (msg.data == 5) {
    Serial.println("Interval 5 detected");   
  }
  else if (msg.data == 6) {
    Serial.println("Interval 6 detected");   
  }
  else if (msg.data == 7) {
    Serial.println("Interval 7 detected");   
  }
}
  
ros::Subscriber<std_msgs::Int16> sub("interval", &interval_msg);

void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(500);
}
