#include <ros/ros.h>
#include <serial/serial.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <sstream>


/// Output content in hexadecimal
void callBack(const std_msgs::UInt8::ConstPtr& msg){

    ROS_INFO("Received message: 0x%x", msg->data);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "serial sub demo");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("serial_data", 100, callBack);

    ros::spin();

    return 0;
}