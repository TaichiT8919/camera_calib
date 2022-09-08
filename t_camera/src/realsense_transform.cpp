#include <ros/ros.h>
#include <string>
#include <fstream>
#include <sstream>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL specific includes
// #include <pcl/ros/conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>

static const double pi = 3.141592653589793;
static const std::string CAM_1_SUB_TOPIC = "/cam_1/depth/color/points";
static const std::string CAM_2_SUB_TOPIC = "/cam_2/depth/color/points";
// static const std::string CAM_3_SUB_TOPIC = "/cam_3/depth/color/points";
// static const std::string CAM_4_SUB_TOPIC = "/cam_4/depth/color/points";

static const bool binary_ = 0;
static const bool compressed_ = 0;

static std::string prefix_; // directory name for save data
static std::string fixed_frame_;  // If set, the transform from the fixed frame to the frame of the point cloud is written to the VIEWPOINT entry of the pcd file. 
static tf2_ros::Buffer tf_buffer_;

ros::Publisher rs1_pub;
ros::Publisher rs2_pub;


sensor_msgs::PointCloud2 rs1_pc;
sensor_msgs::PointCloud2 rs2_pc;
sensor_msgs::PointCloud2 rs3_pc;
sensor_msgs::PointCloud2 rs4_pc;

void rs1_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  rs1_pc = *input;
  // rs1_pc.header.frame_id = "world_link";
  // std::cout << rs1_pc << std::endl;
}

void rs2_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  rs2_pc = *input;
  rs2_pc.header.frame_id = rs1_pc.header.frame_id;
  // rs2_pc.header.frame_id = "world_link";
  // std::cout << rs2_pc << std::endl;
}

// void rs3_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
//   rs3_pc = *input;
//   rs3_pc.header.frame_id = rs1_pc.header.frame_id;
//   // rs1_pc.header.frame_id = "world_link";
//   // std::cout << rs1_pc << std::endl;
// }

// void rs4_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
//   rs4_pc = *input;
//   rs4_pc.header.frame_id = rs1_pc.header.frame_id;
//   // rs2_pc.header.frame_id = "world_link";
//   // std::cout << rs2_pc << std::endl;
// }

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "realsense_transform");
  ros::NodeHandle nh;
  ros::Rate rate(1);
  ros::Rate loop_rate(10);

  // transform listener
  tf::TransformListener tf_listener;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber rs1_sub = nh.subscribe (CAM_1_SUB_TOPIC, 1, rs1_cloud_cb);
  ros::Subscriber rs2_sub = nh.subscribe (CAM_2_SUB_TOPIC, 1, rs2_cloud_cb);
  // ros::Subscriber rs3_sub = nh.subscribe (CAM_3_SUB_TOPIC, 1, rs3_cloud_cb);
  // ros::Subscriber rs4_sub = nh.subscribe (CAM_4_SUB_TOPIC, 1, rs4_cloud_cb);

  // Create a ROS publisher for the output point cloud
  rs1_pub = nh.advertise<sensor_msgs::PointCloud2> ("/rs1/depth/color/transformed/points", 1);
  rs2_pub = nh.advertise<sensor_msgs::PointCloud2> ("/rs2/depth/color/transformed/points", 1);

  // Setting for importing transform matrix 2 to 1
  std::string input_csv_file_path_12  = "/home/rllab/catkin_ws/src/t_hitz_crane/t_camera/config/transform_matrix_12.csv";
  std::ifstream ifs_csv_file_12(input_csv_file_path_12);
  std::string line;
  const std::string delim = " ";
  int col;
  int row = 0;
  Eigen::Matrix4f transform_12 = Eigen::Matrix4f::Identity();
  while (getline(ifs_csv_file_12, line)){
    col = 0;
    for (std::string::size_type spos, epos = 0;
    (spos = line.find_first_not_of(delim, epos)) != std::string::npos;){

      std::string token = line.substr(spos,(epos = line.find_first_of(delim, spos))-spos);

      // convert str to float
      transform_12(row, col++) = stof(token);

      // std::cout << stof(token) << " ";
    }
    ++row;
  }

  // // Setting for importing transform matrix 3 to 1
  // std::string input_csv_file_path_13  = "/home/rllab/catkin_ws/src/t_hitz_crane/t_camera/config/transform_matrix_13.csv";
  // std::ifstream ifs_csv_file_13(input_csv_file_path_13);
  // line;
  // col;
  // row = 0;
  // Eigen::Matrix4f transform_13 = Eigen::Matrix4f::Identity();
  // while (getline(ifs_csv_file_13, line)){
  //   col = 0;
  //   for (std::string::size_type spos, epos = 0;
  //   (spos = line.find_first_not_of(delim, epos)) != std::string::npos;){

  //     std::string token = line.substr(spos,(epos = line.find_first_of(delim, spos))-spos);

  //     // convert str to float
  //     transform_13(row, col++) = stof(token);

  //     // std::cout << stof(token) << " ";
  //   }
  //   ++row;
  // }

  // // Setting for importing transform matrix 4 to 1
  // std::string input_csv_file_path_14  = "/home/rllab/catkin_ws/src/t_hitz_crane/t_camera/config/transform_matrix_14.csv";
  // std::ifstream ifs_csv_file_14(input_csv_file_path_14);
  // line;
  // col;
  // row = 0;
  // Eigen::Matrix4f transform_14 = Eigen::Matrix4f::Identity();
  // while (getline(ifs_csv_file_14, line)){
  //   col = 0;
  //   for (std::string::size_type spos, epos = 0;
  //   (spos = line.find_first_not_of(delim, epos)) != std::string::npos;){

  //     std::string token = line.substr(spos,(epos = line.find_first_of(delim, spos))-spos);

  //     // convert str to float
  //     transform_14(row, col++) = stof(token);

  //     // std::cout << stof(token) << " ";
  //   }
  //   ++row;
  // }

  // Setting for importing transform matrix of psudo pit
  std::string input_csv_file_path2 = "/home/rllab/catkin_ws/src/t_hitz_crane/t_camera/config/psudo_pit_env.csv";
  std::ifstream ifs_csv_file2(input_csv_file_path2);
  line;
  col;
  row = 0;
  Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
  while (getline(ifs_csv_file2, line)){
    col = 0;
    for (std::string::size_type spos, epos = 0;
    (spos = line.find_first_not_of(delim, epos)) != std::string::npos;){

      std::string token = line.substr(spos,(epos = line.find_first_of(delim, spos))-spos);

      // convert str to float
      transform2(row, col++) = stof(token);

      // std::cout << stof(token) << " ";
    }
    ++row;
  }

  // rate.sleep();
  
  // sensor_msgs::PointCloud2 tf_pc1;
  sensor_msgs::PointCloud2 tf_pc2;
  sensor_msgs::PointCloud2 tf_pc3;
  sensor_msgs::PointCloud2 tf_pc4;
  sensor_msgs::PointCloud2 published_pc;

  pcl::PCLPointCloud2 pcl_pc;


  while (ros::ok())
  {
    // this code transforms rs2_pc to tf_pc2 by matrix, trasnform1.
    pcl_ros::transformPointCloud(transform_12, rs2_pc, tf_pc2);
    // pcl_ros::transformPointCloud(transform_13, rs3_pc, tf_pc3);
    // pcl_ros::transformPointCloud(transform_14, rs4_pc, tf_pc4);
    // tf_pc2.header.frame_id = "cam_1_link";

    // pcl::concatenatePointCloud(rs1_pc, tf_pc4, published_pc);
    pcl::concatenatePointCloud(rs1_pc, tf_pc2, published_pc);
    // pcl::concatenatePointCloud(published_pc, tf_pc3, published_pc);
    // pcl::concatenatePointCloud(published_pc, tf_pc4, published_pc);

    // pcl_ros::transformPointCloud(transform2, published_pc, published_pc);

		rs1_pub.publish(published_pc);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
}



