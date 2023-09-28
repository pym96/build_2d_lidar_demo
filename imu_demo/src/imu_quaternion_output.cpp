#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "imu_demo/imu_quaternion_output.hpp"

const bool debug_ = true;

void debug_serial_output_1(const uint8_t& data){
    ROS_INFO("Received message: 0x%x", data);
}


void debug_serial_output_vector(const std::vector<uint8_t>& vec){
    ROS_INFO("Current frame is: ");

    for(auto x: vec)
        ROS_INFO("%d \t", x);
    
}

void debug_serial_output_vector_hexadecimal(const std::vector<uint8_t>& vec){
    ROS_INFO("Current frame is: ");

    for(auto x: vec)
        ROS_INFO("0x%x \t", x);
    
}


void debug_serial_print_out_receive_packet(const ReceivePacket& packet){
    ROS_INFO("Received packet:");
    ROS_INFO("Head: 0x%02X", packet.head);
    ROS_INFO("Data Type: %d", packet.data_type);
    ROS_INFO("Quaternion[0]: 0x%02X%02X", packet.quaternion0_high, packet.quaternion0_low);
    ROS_INFO("Quaternion[1]: 0x%02X%02X", packet.quaternion1_high, packet.quaternion1_low);
    ROS_INFO("Quaternion[2]: 0x%02X%02X", packet.quaternion2_high, packet.quaternion2_low);
    ROS_INFO("Quaternion[3]: 0x%02X%02X", packet.quaternion3_high, packet.quaternion3_low);
    ROS_INFO("Check Sum: 0x%02X", packet.check_sum);
}

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
        tf::TransformBroadcaster tf_broadcaster;

        uint8_t head = 0;
        uint8_t data_type = 0;  // Choosing quaterion.

        int16_t roll = 0;
        int16_t pitch = 0;
        int16_t yaw = 0;
        
        while (ros::ok()) {
            ros::spinOnce();
                
            // Read the data and convert to a hexadecimal string
            ser.read(&head, 1);

            // Frame head found
            if(head == 0x55){
                ser.read(&data_type,1);

                std::vector<uint8_t> packet;
                packet.reserve(sizeof(ReceivePacket) - 2);
             
                // Read the 9 bytes in a line 
                if(data_type == angle){
                    ser.read(packet,(sizeof(ReceivePacket) - 2));
                

                    packet.insert(packet.begin(),data_type);
                    packet.insert(packet.begin(), head);
                    // ROS_INFO("Current sizeof(packet) is: %ld",sizeof(packet));
                    ReceivePacket data = fromVector(packet);

                    if(debug_) {
                         debug_serial_output_vector_hexadecimal(packet);
                         debug_serial_print_out_receive_packet(data);
                    }

                    // Getthing roll yaw, and pitch, from the gyroscope constract
                    roll = ((data.quaternion0_high << 8) | data.quaternion0_low) / 32768 * 180;
                    pitch = ((data.quaternion1_high << 8) | data.quaternion1_low) / 32768 * 180;
                    yaw = static_cast<int16_t>((data.quaternion2_high << 8) | data.quaternion2_low);

                    double yaw_ = yaw * 0.0054931640625f;
                    
                    ROS_INFO("Current roll is: %u, pitch is: %u, yaw is: %f",roll, pitch, yaw_); 
                    tf::Quaternion quaternion;
                    quaternion.setRPY(0, 0, static_cast<double>(yaw_ * M_PI / 180));
                    
                    // Create a transform message
                    geometry_msgs::TransformStamped transform_stamped;
                    transform_stamped.header.stamp = ros::Time::now();
                    transform_stamped.header.frame_id = "base_link"; // Parent frame link
                    transform_stamped.child_frame_id = "child_link"; // Child frame link
                    
                    geometry_msgs::Quaternion msg_quaternion;
                    msg_quaternion.x = quaternion.x();
                    msg_quaternion.y = quaternion.y();
                    msg_quaternion.z = quaternion.z();
                    msg_quaternion.w = quaternion.w();

                    transform_stamped.transform.rotation = msg_quaternion;

                    tf_broadcaster.sendTransform(transform_stamped);
                }
            }
            
        }
    } else {
        ROS_ERROR_STREAM("Serial port is not open.");
        return 1;
    }

    return 0;
}


