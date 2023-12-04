#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

const std::string pcd_path = "/home/dan/learn/buld2d_demo/src/ma_navigation/pcl2pcd/map/output.pcd";


int main(int argc, char** argv){

    // Load point cloud file
    PointCloudType::Ptr cloud(new PointCloudType);
    pcl::io::loadPCDFile(pcd_path, *cloud);

    if(cloud->empty()){
        ROS_WARN("Cannot load point cloud file");
        return -1;
    }

    ROS_INFO("Cloud points: %zu", cloud->size());

    // visualization
    pcl::visualization::PCLVisualizer viewer("cloud viewer");
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> handle(cloud, "z");
    viewer.addPointCloud<PointType>(cloud, handle);
    viewer.spin();
    
    return 0;
}