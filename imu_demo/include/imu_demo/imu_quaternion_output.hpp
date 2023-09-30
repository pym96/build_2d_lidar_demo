#ifndef __IMU_QUATERNION_OUTPUT_HPP_
#define __IMU_QUATERNION_OUTPUT_HPP_

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>  

#include <cinttypes>
#include <cstdint>
#include <memory>
#include <vector>


extern const uint8_t time_stamp = 0x50;
extern const uint8_t linear_acceleration = 0x51;
extern const uint8_t angular_velocity = 0x52;
extern const uint8_t angle = 0x53;
extern const uint8_t quaternion = 0x59;

/**
 * ReceivePacket: For manipulating gyroscope data from serial
 * 
*/
struct ReceivePacket{
    uint8_t head = 0x55;
    uint8_t data_type = quaternion; // Quaternion default

    uint8_t quaternion0_low;
    uint8_t quaternion0_high;

    uint8_t quaternion1_low;
    uint8_t quaternion1_high;
    
    uint8_t quaternion2_low;
    uint8_t quaternion2_high;

    uint8_t quaternion3_low;
    uint8_t quaternion3_high;

    char check_sum : 8;

}__attribute__((packed));

/**
 * Alter vector above into struct and utilize it.
*/
inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

/**
 * Alter struct object above into vector and utilize it.
*/
// inline std::vector<uint8_t> toVector(const SendPacket & data)
// {
//   std::vector<uint8_t> packet(sizeof(SendPacket));
//   std::copy(
//     reinterpret_cast<const uint8_t *>(&data),
//     reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
//   return packet;
// }


#endif