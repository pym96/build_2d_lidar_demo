#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <cinttypes>
#include <cstdint>
#include <memory>

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_demo_node");
    ros::NodeHandle nh;

    // Set the serial port name and baud rate (adjust as needed)
    std::string serial_port = "/dev/ttyUSB0";
    int baud_rate = 115200;

    serial::Serial ser;
    try {
        ser.setPort(serial_port);
        ser.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open the serial port: " << e.what());
        return 1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized.");

        ros::Publisher pub = nh.advertise<std_msgs::UInt8>("serial_data", 100);

        while (ros::ok()) {
            ros::spinOnce();
                
            // Read the data and convert to a hexadecimal string

            uint8_t data = 0;
            ser.read(&data, 1);

            std_msgs::UInt8 msg;

            msg.data = data;

            pub.publish(msg);
            
        }
    } else {
        ROS_ERROR_STREAM("Serial port is not open.");
        return 1;
    }

    return 0;
}


