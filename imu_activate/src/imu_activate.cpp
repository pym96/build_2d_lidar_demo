#include <ros/ros.h>
#include <serial/serial.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>


static bool is_activiated = false;

void activateCallBack(){
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gyroscope_activation_node");
    ros::NodeHandle nh;

    // Set the serial port name and baud rate (adjust as needed)
    const std::string serial_port = "/dev/ttyUSB0";
    const int baud_rate = 9600;

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

        // Create a message to send the hexadecimal values
        std_msgs::UInt8MultiArray msg;
        msg.data.push_back(0xFF);
        msg.data.push_back(0xAA);
        msg.data.push_back(0x69);
        msg.data.push_back(0x88);
        msg.data.push_back(0xB5);

        // Publish the message to the serial port
        ser.write(msg.data);

        // Close the serial port after sending the data (if needed)
        ser.close();
    } else {
        ROS_ERROR_STREAM("Serial port is not open.");
        return 1;
    }

    return 0;
}
