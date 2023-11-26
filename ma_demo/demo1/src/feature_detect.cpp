#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

#include <gflags/gflags.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "tf_demo_node");

    ros::NodeHandle nh;

    tf::TransformBroadcaster tf_broadcaster;

    // Create a transform message
        geometry_msgs::TransformStamped transform_stamped;

        while (ros::ok()) {
            ros::spinOnce();

                    transform_stamped.header.stamp = ros::Time::now();
                    transform_stamped.header.frame_id = "map"; // Parent frame link
                    transform_stamped.child_frame_id = "base_link"; // Child frame link
                    
                    geometry_msgs::Quaternion msg_quaternion;
                    msg_quaternion.x = 0;
                    msg_quaternion.y = 0;
                    msg_quaternion.z = 0;
                    msg_quaternion.w = 0;

                    transform_stamped.transform.rotation = msg_quaternion;

                    tf_broadcaster.sendTransform(transform_stamped);       
        }
    

    return 0;
}