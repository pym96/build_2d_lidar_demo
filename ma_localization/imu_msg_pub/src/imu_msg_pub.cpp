#include "imu_demo/imu_quaternion_output.hpp"

#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>

#include <gflags/gflags.h>

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

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);

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
        sensor_msgs::Imu imu;

        uint8_t head = 0;
        uint8_t data_type = 0;  // Choosing quaterion.

        int16_t l_acc_x = 0;
        int16_t l_acc_y = 0;
        int16_t l_acc_z = 0;

        int16_t a_v_x = 0;
        int16_t a_v_y = 0;
        int16_t a_v_z = 0;

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
                if (data_type == linear_acceleration){
                    ser.read(packet,(sizeof(ReceivePacket) - 2));

                    packet.insert(packet.begin(),data_type);
                    packet.insert(packet.begin(), head);
                    // ROS_INFO("Current sizeof(packet) is: %ld",sizeof(packet));
                    ReceivePacket data_from_packet = fromVector(packet);

                    #if 1
                    if(debug_) {
                         debug_serial_output_vector_hexadecimal(packet);
                         debug_serial_print_out_receive_packet(data_from_packet);
                    }
                    #endif

                    l_acc_x = static_cast<int16_t>((data_from_packet.quaternion0_high << 8) | data_from_packet.quaternion0_low);
                    l_acc_y = static_cast<int16_t>((data_from_packet.quaternion1_high << 8) | data_from_packet.quaternion1_low);
                    l_acc_z = static_cast<int16_t>((data_from_packet.quaternion2_high << 8) | data_from_packet.quaternion2_low);
                    
                    double l_acc_x_ = l_acc_x / 32768. * (16 * 9.8);
                    double l_acc_y_ = l_acc_y / 32768. * (16 * 9.8);
                    double l_acc_z_ = l_acc_z / 32768. * (16 * 9.8);
                    
                    imu.header.stamp = ros::Time::now();
                    imu.header.frame_id = "base_link";

                    imu.linear_acceleration.x = l_acc_x_;
                    imu.linear_acceleration.y = l_acc_y_;
                    imu.linear_acceleration.z = l_acc_z_;

                }else if(data_type == angular_velocity){
                    ser.read(packet,(sizeof(ReceivePacket) - 2));

                    packet.insert(packet.begin(),data_type);
                    packet.insert(packet.begin(), head);
                    // ROS_INFO("Current sizeof(packet) is: %ld",sizeof(packet));
                    ReceivePacket data_from_packet = fromVector(packet);

                    #if 1
                    if(debug_) {
                         debug_serial_output_vector_hexadecimal(packet);
                         debug_serial_print_out_receive_packet(data_from_packet);
                    }
                    #endif

                    a_v_x = static_cast<int16_t>((data_from_packet.quaternion0_high << 8) | data_from_packet.quaternion0_low);
                    a_v_y = static_cast<int16_t>((data_from_packet.quaternion1_high << 8) | data_from_packet.quaternion1_low);
                    a_v_z = static_cast<int16_t>((data_from_packet.quaternion2_high << 8) | data_from_packet.quaternion2_low);
                    
                    double a_v_x_ = a_v_x / 32768. * 2000.;
                    double a_v_y_ = a_v_y / 32768. * 2000.;
                    double a_v_z_ = a_v_z / 32768. * 2000.;

                    imu.angular_velocity.x = a_v_x_;
                    imu.angular_velocity.y = a_v_y_;
                    imu.angular_velocity.z = a_v_z_;
                }else if(data_type == angle){
                    
                    ser.read(packet,(sizeof(ReceivePacket) - 2));

                    packet.insert(packet.begin(),data_type);
                    packet.insert(packet.begin(), head);
                    // ROS_INFO("Current sizeof(packet) is: %ld",sizeof(packet));
                    ReceivePacket data_from_packet = fromVector(packet);

                    #if 1
                    if(debug_) {
                         debug_serial_output_vector_hexadecimal(packet);
                         debug_serial_print_out_receive_packet(data_from_packet);
                    }
                    #endif

                    // Getthing roll yaw, and pitch, from the gyroscope constract
                    roll = static_cast<int16_t>((data_from_packet.quaternion0_high << 8) | data_from_packet.quaternion0_low);
                    pitch = static_cast<int16_t>((data_from_packet.quaternion1_high << 8) | data_from_packet.quaternion1_low);
                    yaw = static_cast<int16_t>((data_from_packet.quaternion2_high << 8) | data_from_packet.quaternion2_low);

                    double roll_ = roll * 0.0054931640625f;
                    double pitch_ = pitch * 0.0054931640625f;
                    double yaw_ = yaw * 0.0054931640625f;

                    tf::Quaternion quaternion;
                    quaternion.setRPY(static_cast<double>(roll_ * M_PI / 180), static_cast<double>(pitch_ * M_PI / 180), static_cast<double>(yaw_ * M_PI / 180));
                    
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

                    imu.orientation.w = quaternion.w();
                    imu.orientation.x = quaternion.x();
                    imu.orientation.y = quaternion.y();
                    imu.orientation.z = quaternion.z();
                }
                else{
                // TODO: publish imu data
                    imu_pub.publish(imu); 

                }
            }
        }
    } else {
        ROS_ERROR_STREAM("Serial port is not open.");
        return 1;
    }

    return 0;
}


