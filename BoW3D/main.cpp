#include <ros/ros.h>
#include <string>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <iomanip>
#include "LinK3D_Extractor.h"
#include "BoW3D.h"
#include "Preprocess.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace BoW3D;
 string dataset_name_ ;
 string save_path_;

ros::Publisher pub_scan;
ros::Publisher pub_edge;
ros::Publisher pub_keypoints;
ros::Publisher pub_path_;

nav_msgs::Path path_;
sensor_msgs::PointCloud2 publish_clouds(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud)
{   
    sensor_msgs::PointCloud2 Msg;
    pcl::toROSMsg(*input_cloud, Msg);
    Msg.header.frame_id = "lidar";
    return Msg;
}
geometry_msgs::Pose eigenMatrixToPose(const Eigen::Matrix4d &eigen_matrix)
{
    geometry_msgs::Pose pose;

    // 提取平移信息
    pose.position.x = eigen_matrix(0, 3);
    pose.position.y = eigen_matrix(1, 3);
    pose.position.z = eigen_matrix(2, 3);

    // 从旋转矩阵中提取四元数
    Eigen::Quaterniond quat(eigen_matrix.block<3, 3>(0, 0));
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
}
void visualize_path( nav_msgs::Path &path, const Eigen::Matrix4d &poses)
{
    geometry_msgs::PoseStamped pose_stamped;
   
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = eigenMatrixToPose(poses);
    path.header = pose_stamped.header;
    path.poses.push_back(pose_stamped);
}

void visualize_raw_ptcloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) 
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*input_cloud, *raw_cloud);

    // 然后设置强度值
    for (auto& point : *raw_cloud) {
        point.intensity = 10.0f;  // 默认强度设为0，可根据需要修改
    }
    
    sensor_msgs::PointCloud2 scan_msg=publish_clouds(raw_cloud);
    pub_scan.publish(scan_msg);  
}
void visualize_AggregationKeypoints( std::vector<pcl::PointXYZI> AggregationKeypoints)
{   
    pcl::PointCloud<pcl::PointXYZI>::Ptr Keypoints(new pcl::PointCloud<pcl::PointXYZI>());
    for (size_t i = 0; i < AggregationKeypoints.size(); ++i) {
        const pcl::PointXYZI& pt = AggregationKeypoints[i];
           pcl::PointXYZI points;
           points.x= pt.x;
           points.y= pt.y;
           points.z= pt.z; 
           points.intensity=100; 
           Keypoints->push_back(points); 
       }
       sensor_msgs::PointCloud2 Keypoints_msg=publish_clouds(Keypoints);
       pub_keypoints.publish(Keypoints_msg);  

}
void visualize_Edgepoints( ScanEdgePoints ClusterEdgeKeypoints)
{   
    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZI>());
   
    int numCluster = ClusterEdgeKeypoints.size();

    for(int i = 0; i < numCluster; i++)
    {
        int numPt = ClusterEdgeKeypoints[i].size();

        for(int j = 0; j < numPt; j++)
        {   
            pcl::PointXYZI edge_points;
            PointXYZSCA pt = ClusterEdgeKeypoints[i][j];
            edge_points.x= pt.x;
            edge_points.y= pt.y;
            edge_points.z= pt.z; 
            edge_points.intensity=50;  
            edge_cloud->push_back(edge_points);            
        }

    }
    sensor_msgs::PointCloud2 edge_msg=publish_clouds(edge_cloud);
    pub_edge.publish(edge_msg);    

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BoW3D");
    ros::NodeHandle nh("~"); 
    pub_scan = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/curr_scan", 100);
    pub_edge = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/curr_edge", 100);
    pub_keypoints = nh.advertise<sensor_msgs::PointCloud2>("/velodyne/valid_edge", 100);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path/path_ground_path", 100);

    Preprocess *preprocess_data= new Preprocess(&nh);
    vector<pcl::Indices> GT_loop= preprocess_data->get_GT_loop_closure(preprocess_data->GTposes_folder_); 
    vector<Eigen::Matrix4d> GT_poses= preprocess_data->visualize_ground_truth(preprocess_data->GTposes_folder_); 
    
    BoW3D::LinK3D_Extractor* pLinK3dExtractor = new BoW3D::LinK3D_Extractor(&nh); 
    BoW3D::BoW3D* pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor,&nh);

    size_t frame_id = 1;


    ros::Rate LiDAR_rate(100); //LiDAR frequency 10Hz
    while (ros::ok())
    {   
        bool loop_flag=0;
        vector<int>::iterator it;  
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
       
        current_cloud=preprocess_data->get_lidar_data(frame_id);
        Frame* pCurrentFrame = new Frame(pLinK3dExtractor,current_cloud); 
        
        visualize_raw_ptcloud(current_cloud); 
        visualize_Edgepoints(pCurrentFrame->mClusterEdgeKeypoints);
        visualize_AggregationKeypoints(pCurrentFrame->mvAggregationKeypoints);
        
        //Eigen::Matrix4d curr_poses=GT_poses[frame_id];
        //visualize_path(path_,curr_poses);
        //pub_path_.publish(path_);

        if(pCurrentFrame->mnId < (long unsigned int)pBoW3D->num_nodes_exclude)
        {
            pBoW3D->update(pCurrentFrame);  
        }
        else
        {                
            int loopFrameId = -1;
            double matched_count = 0;
            Eigen::Matrix3d loopRelR;
            Eigen::Vector3d loopRelt;
            //auto start = ros::Time::now().toSec();
            pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt,matched_count); 
           // auto end = ros::Time::now().toSec();
            pBoW3D->update(pCurrentFrame);               
            //cout<<"  "<<"\033[1;32mFrame"<<pCurrentFrame->mnId<<" "<<"costs"<<1000*(end-start)<<"ms\033[0m"<<endl;
            
            it = std::find(GT_loop[frame_id].begin(),GT_loop[frame_id].end(),loopFrameId);
            if(it!=GT_loop[frame_id].end()) loop_flag=1;
            
            std::ofstream save_results(preprocess_data->save_folder_, std::ios::app);
            save_results.setf(std::ios::fixed, std::ios::floatfield);  
            save_results.precision(5);

            if(loopFrameId == -1)
            {
             cout <<"  "<< "\033[1;32mFrame"<< pCurrentFrame->mnId+1<<" "<< "has no Loop-closure Frame\033[0m" << endl;
             
             save_results << pCurrentFrame->mnId+1 << " " <<loopFrameId<< " "<<1<< " "<< loop_flag <<endl; 
             
            }
            else
            {

             cout <<"  "<<"\033[1;32mFrame" << pCurrentFrame->mnId+1<<" "<<"has Loop-closure Frame\033[0m"<<loopFrameId+1 << endl; 
        
             save_results << pCurrentFrame->mnId+1 << " " <<loopFrameId+1 << " "<< matched_count<< " "<< loop_flag <<endl; 
             
            }
            save_results.close(); 
        }                       
        
        frame_id ++;
        ros::spinOnce();
        LiDAR_rate.sleep();
    }

    return 0;
}

