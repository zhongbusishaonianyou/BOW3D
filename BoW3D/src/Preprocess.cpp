#include "Preprocess.h"
#include <cmath>
using namespace std;
typedef pcl::PointXYZ  PointType;

Preprocess::Preprocess(ros::NodeHandle* nh_load_data) : nh_load_data(*nh_load_data) {
    
    nh_load_data->param("/bow3d/revisit_threshold", revisit_thres,4.0f);
    nh_load_data->param<std::string>("/dataset/sequence_name", sequence," 05");
    nh_load_data->param<std::string>("/dataset/dataset_path",dataset_path_, "/");
    nh_load_data->param<std::string>("/dataset/pose_path", GTposes_path_, "/");
    nh_load_data->param<std::string>("/dataset/GTloop_path", GTloop_path_,"/" );
    nh_load_data->param<std::string>("/dataset/results_path", save_path_,"/" );
    
    dataset_folder_ = dataset_path_+"/"+sequence +"/velodyne/"; 
    GTposes_folder_=GTposes_path_+"/"+sequence +".txt";
    GTloop_folder_=GTloop_path_+"/"+"gt_loop_"+ sequence + ".txt";
    save_folder_=save_path_+"/kitti_"+sequence+".txt";
 }
Preprocess::~Preprocess()
{ 

}
pcl::PointCloud<pcl::PointXYZ>::Ptr Preprocess::downsample(vector<float> & lidar_data)
{
    
  // pcl::VoxelGrid<PointType> downSizeFilter;
 //  downSizeFilter.setLeafSize(filter_size, filter_size, filter_size);
  // pcl::PointCloud<PointType>::Ptr current_cloudDS(new pcl::PointCloud<pcl::PointXYZ>());
   pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        for(std::size_t i = 0; i < lidar_data.size(); i += 4)
        {            
            pcl::PointXYZ point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
    
            current_cloud->push_back(point);
        }
          
          //current_cloudDS->clear();
         // downSizeFilter.setInputCloud(current_cloud);
         // downSizeFilter.filter(*current_cloudDS);

          return current_cloud;
}         
std::vector<float> Preprocess::read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file;
    lidar_data_file.open(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    if(!lidar_data_file)
    {
        cout << "Read LIDAR data End..." << endl;
        exit(-1);
    }

    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Preprocess::read_poses_data(const string& pose_path)
{
    std::ifstream pose_ifs(pose_path);
    std::string line;
    pcl::PointCloud<pcl::PointXYZI>::Ptr GTposes(new pcl::PointCloud<pcl::PointXYZI>);

    int index = 1;
    while(getline(pose_ifs, line)) 
    {   
        
        if(line.empty()) break;
        stringstream ss(line);
        float token;
        vector<float> tokens;
       while(ss>>token)
        {
            tokens.push_back(token);
       }
        pcl::PointXYZI curr_poses;
        curr_poses.x = tokens[3];
        curr_poses.y = 0;
        curr_poses.z = tokens[11];
        curr_poses.intensity = index++;
       
        GTposes->push_back(curr_poses);
        
    }
    return GTposes;
}

vector<Eigen::Matrix4d> Preprocess::visualize_ground_truth(const string& pose_path)
{
    std::ifstream pose_ifs(pose_path);
    std::string line;
    vector<Eigen::Matrix4d> matrix_poses;
    while(getline(pose_ifs, line)) 
    {   
        
        if(line.empty()) break;
        stringstream ss(line);
        float token;
        vector<float> tokens;
       while(ss>>token)
        {
            tokens.push_back(token);
       }
        Eigen::Matrix4d curr_poses;

        curr_poses.row(0) << tokens[0], tokens[1], tokens[2],tokens[3];
        curr_poses.row(1) << tokens[4], tokens[5], tokens[6],tokens[7];
        curr_poses.row(2) << tokens[8], tokens[9], tokens[10],tokens[11];
        curr_poses.row(3) << 0, 0, 0,1;

        matrix_poses.push_back(curr_poses);
        
    }
    return matrix_poses;
}


std::vector<vector<int>> Preprocess::calculate_GT_loop(pcl::PointCloud<pcl::PointXYZI>::Ptr & GTposes)
{
    int num_series=GTposes->points.size();
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(GTposes);
    std::vector<vector<int>> GT_loop(num_series+1);
    for(int i = 0; i < num_series; i++)
    {
        
        std::vector<int> pt_index;
        std::vector<float> pt_dis;
        std::vector<int> loop_frame_id;

        int query_frame_id = GTposes->points[i].intensity;
        kdtree.radiusSearch(GTposes->points[i],revisit_thres,pt_index,pt_dis);

        for(int j = 0; j < (int)pt_index.size(); j++)
        {  
          int history_frame=GTposes->points[pt_index[j]].intensity;
           
            if(history_frame == query_frame_id) continue;
               loop_frame_id.push_back(history_frame);
        }
        sort(loop_frame_id.begin(), loop_frame_id.end());
        GT_loop[query_frame_id] = loop_frame_id;
        
    }
    
    std::ofstream save_GTloop(GTloop_folder_);

    for(int i =1; i < (int)GT_loop.size(); i++)
    {
        save_GTloop << i << " ";
        for(int j = 0; j < (int)GT_loop[i].size(); j++)
        {
            save_GTloop << GT_loop[i][j] << " ";
        }
        
        save_GTloop << endl;
    }
    
    return GT_loop;
}
std::vector<vector<int>> Preprocess::get_GT_loop_closure(const string& pose_path)
{
   pcl::PointCloud<pcl::PointXYZI>::Ptr GT_poses = read_poses_data(pose_path);

     return calculate_GT_loop(GT_poses);
} 

 pcl::PointCloud<PointType>::Ptr Preprocess::get_lidar_data(const size_t & ID)
{
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder_ << std::setfill('0') << std::setw(5) << ID << ".bin";
        cout<<"Read:"<<lidar_data_path.str()<<endl;
        vector<float> lidar_data =read_lidar_data(lidar_data_path.str());
        return downsample(lidar_data);
}           
