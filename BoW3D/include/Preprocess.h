#pragma once

#include <ros/ros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree_flann.h>

#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <string>
using namespace std;
class Preprocess
{

public:
      Preprocess(ros::NodeHandle* nh_load_data);
     ~Preprocess();
     vector<float> read_lidar_data(const std::string lidar_data_path);
     pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(vector<float> & lidar_data);
     pcl::PointCloud<pcl::PointXYZI>::Ptr read_poses_data(const string& pose_path);
     std::vector<std::vector<int>> calculate_GT_loop(pcl::PointCloud<pcl::PointXYZI>::Ptr & GTposes);
     std::vector<std::vector<int>> get_GT_loop_closure(const string& pose_path);
     pcl::PointCloud<pcl::PointXYZ>::Ptr get_lidar_data(const size_t & ID);
     vector<Eigen::Matrix4d> visualize_ground_truth(const string& pose_path);

private:
     ros::NodeHandle nh_load_data;
     string dataset_path_;
     string GTposes_path_;
     string GTloop_path_;

     string dataset_folder_;     
     string GTloop_folder_;
     string save_path_;
public:
     string sequence;
     string GTposes_folder_; 
     string save_folder_;
     float revisit_thres;
           
};