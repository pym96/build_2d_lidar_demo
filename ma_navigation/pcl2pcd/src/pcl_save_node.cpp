#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <vector>
#include <ostream>
#include <string>


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Save to .pcd file
    pcl::io::savePCDFile("/home/dan/learn/buld2d_demo/src/ma_navigation/pcl2pcd/map/output.pcd", cloud);
    ROS_INFO("Saved %d data points size: ", static_cast<int> (cloud.size()));
}

void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    // Convert pointcloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Process the cloud data here, maybe I eliminate too high voxel data...

    // Create and open a file
    std::ofstream file;
    file.open("../map/ouput.pgm");

    const int width = 300.;
    const int height = 300.;

    std::vector<std::vector<int>> grid(height, std::vector<int>(width, 0));

    // Write pgm header
    file << "P2\n";
    file << width << " " << height << "\n";
    file << "255\n" ; // Max gray value
    
    // Write pixel data
    for(int i = 0; i < height; ++i){
        for(int j =0 ; j < width; ++j){
            // Writing the grayscale value
            file << grid[i][j] << " ";
        }
    }

    // Close the file
    file.close();
}

void save_pgm2yaml(const std::string& filename, const std::string& yaml_name){

    const float resolution = 0.05; // 5cm per pixel
    float origin_x = -10.0, origin_y = -10.0, origin_yaw = 0.0; // Example origin
    int negate = 0; // 0 or 1
    float occupied_thresh = 0.65;
    float free_thresh = 0.196;

    // Create and oepn a file
    std::ofstream yaml_file(yaml_name);

    // Write data to the file
    yaml_file << "image: " << filename << "\n";
    yaml_file << "resolution: " << resolution << "\n";
    yaml_file << "origin: [" << origin_x << ", " << origin_y << ", " << origin_yaw << "]\n";
    yaml_file << "negate: " << negate << '\n';
    yaml_file << "occupied_thresh" << occupied_thresh << "\n";
    yaml_file << "free_thresh" << free_thresh << '\n';

    // Close the file
    yaml_file.close();
}


int main(int argc, char ** argv){


    ros::init(argc, argv, "pcl_save_node");

    ros::NodeHandle nh;
    
    // Create a ROS subscriber for point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/cloud_registered", 1, cloud_cb);

    ros::spin();

    return 0;
}