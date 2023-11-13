#ifndef __HEAD_IMU_PREPROCESSING__
#define __HEAD_IMU_PREPROCESSING__


#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include "so3_math.h"
#include <Eigen/Eigen>
#include "common_lib.h"
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"



#define MAX_INI_COUNT (10)

/**
 * IMU Process
*/
namespace imu_preprocess{

    class ImuProcess{

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        public:
            explicit ImuProcess();
            ~ImuProcess();
        
        private:
            ImuProcess(const ImuProcess&);
            ImuProcess& operator=(const ImuProcess&);

        public:
            void reset_imu();
            void reset_imu(const double& start_time_stamp, const sensor_msgs::ImuConstPtr& last_imu);
            void set_extrinsic(const V3D& translation, const V3D& rotation);
            void set_extrinsic(const MD(4, 4)& T);
            void set_gyr_conv(const V3D& scaler);
            void set_acc_conv(const V3D& scaler);
            void set_gyr_bias_cov(const V3D& b_g);
            void set_acc_bias_cov(const V3D& b_a);

            Eigen::Matrix<double, 12, 12> Q;
            
            void process(const MeasureGroup&  meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, PointCloudXYZI &pcl_in_out);

            

    };
}

#endif 