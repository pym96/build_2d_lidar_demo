#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <your_package_name/ImuData.h>  // Include your custom message header

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_demo_node");
    ros::NodeHandle nh;

    // ... (Initialization and serial communication code)

    // Create a publisher for your custom IMU data message
    ros::Publisher imu_data_pub = nh.advertise<your_package_name::ImuData>("imu_data", 100);

    while (ros::ok()) {
        // ... (Serial data processing and transformation code)

        // Publish the custom IMU data message
        your_package_name::ImuData imu_msg;
        imu_msg.roll = roll;
        imu_msg.pitch = pitch;
        imu_msg.yaw = yaw;
        imu_msg.angular_velocity_x = angular_velocity_x;
        imu_msg.angular_velocity_y = angular_velocity_y;
        imu_msg.angular_velocity_z = angular_velocity_z;
        imu_msg.linear_acceleration_x = linear_acceleration_x;
        imu_msg.linear_acceleration_y = linear_acceleration_y;
        imu_msg.linear_acceleration_z = linear_acceleration_z;

        imu_data_pub.publish(imu_msg);

        // ... (Other code)

        ros::spinOnce();
    }

    return 0;
}
