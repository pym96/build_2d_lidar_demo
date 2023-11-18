#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Node to send Twist command for a 2d navigation robot
int main(int argc, char** argv){

    ros::init(argc, argv, "Open loop control");
    ros::NodeHandle nh;

    ros::Publisher twist_commander 
                                = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
    
    const double sample_dt = 0.01; // Specify a sample period of 10ms
    const double speed = 1.0; // 1m / s
    const double yaw_rate = 0.5; // 0.5 rad/s yaw rate command
    const double time_3_sec = 3.0; // Should moving 3 meters ro 1.5 rad in 3 seconds

    geometry_msgs::Twist twist_cmd; // This is the message type required to send twist
    
    // All parameters are 0 by default
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;

    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = 0.0;

    ros::Rate loop_timer(1 / sample_dt); // Create a ros object from the ros Rate class
                                        // Set 100hz here
    double timer = 0.0;

    for(int i = 0; i < 10; ++i){
        twist_commander.publish(twist_cmd);
        loop_timer.sleep();
    }

    twist_cmd.linear.x = speed;
    while(timer < time_3_sec){
        twist_commander.publish(twist_cmd);
        timer += sample_dt;
        loop_timer.sleep();
    }


    twist_cmd.angular.z = 0.0; // Stop spinning in place
    twist_cmd.linear.x = speed; // Move forward again
    timer = 0.0;    // Reset the timer

    while(timer < time_3_sec){
        twist_commander.publish(twist_cmd);
        timer += sample_dt;
        loop_timer.sleep();
    }

    // Halt the motion
    twist_cmd.angular.x = 0.0;
    twist_cmd.linear.x = 0.0;
    for(int i = 0; i < 10; ++i){
        twist_commander.publish(twist_cmd);
        loop_timer.sleep();
    }

    // Done commanding the robot; node runs to completion
}