#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Livox custom message
#include "livox_ros_driver/CustomMsg.h"

// Use pcl::PointXYZINormal to retain intensity/curvature information
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

//------------------ Global Variables ------------------//

// Whether to perform global coordinate transformation (determined by the "global" parameter)
bool global_coord = false;

// Pose (assumed to represent "sensor pose in the map coordinate frame")
geometry_msgs::PoseStamped current_pose;
bool pose_received = false;  

// Used to merge multiple frames
uint64_t TO_MERGE_CNT = 1; 
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

// Topic names loaded via parameters
std::string input_pointcloud_topic;
std::string input_pose_topic;
std::string output_pointcloud_topic;

// Publisher
ros::Publisher pub_pcl_out;

/**
 * @brief Pose callback function: subscribe to input_pose_topic
 */
void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  current_pose = *pose_msg;
  pose_received = true;
}

/**
 * @brief Transform point cloud from "sensor frame" to "map frame" (using Eigen)
 * @param[in,out] pcl_in  Point cloud to be transformed (modified in-place)
 */
void transformPointCloud(PointCloudXYZI& pcl_in) {
  if (!pose_received) {
    ROS_WARN_THROTTLE(5.0, 
      "[livox_repub_eigen_param] Pose not received yet, skip transform.");
    return;
  }

  // Extract translation
  Eigen::Vector3d t(
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      current_pose.pose.position.z
  );

  // Extract quaternion (note the order: Eigen uses (w, x, y, z) for construction)
  Eigen::Quaterniond q(
      current_pose.pose.orientation.w,
      current_pose.pose.orientation.x,
      current_pose.pose.orientation.y,
      current_pose.pose.orientation.z
  );

  // Construct the transformation matrix T_sensor_to_map
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translation() = t;
  T.linear() = q.normalized().toRotationMatrix();

  // Iterate over the point cloud and apply coordinate transformation
  for (auto& pt : pcl_in.points) {
    Eigen::Vector3d v(pt.x, pt.y, pt.z);
    Eigen::Vector3d v_map = T * v; 
    pt.x = static_cast<float>(v_map.x());
    pt.y = static_cast<float>(v_map.y());
    pt.z = static_cast<float>(v_map.z());
  }
}

/**
 * @brief Livox custom point cloud callback function
 */
void LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  // Add the current frame to the buffer
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) {
    return;
  }

  // 1) Create a PCL point cloud container
  PointCloudXYZI pcl_in;

  // 2) Merge multiple frames
  for (auto& livox_msg : livox_data) {
    if (livox_msg->points.empty()) continue;

    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;

      float s = livox_msg->points[i].offset_time 
                / static_cast<float>(time_end);

      pt.intensity = livox_msg->points[i].line 
                   + livox_msg->points[i].reflectivity / 10000.0f;
      pt.curvature = s * 0.1f;

      pcl_in.push_back(pt);
    }
  }
  livox_data.clear();

  // 3) Perform global coordinate transformation if enabled
  if (global_coord) {
    // Sensor -> Map
    transformPointCloud(pcl_in);
  }

  // 4) Retrieve timestamp
  unsigned long timebase_ns = livox_msg_in->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  // 5) Convert to ROS standard point cloud message
  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);

  // If global coordinate transformation is applied, set frame to "map"; otherwise, use "map" or other frame
  if(global_coord) {
    pcl_ros_msg.header.frame_id = "map";
  } else {
    pcl_ros_msg.header.frame_id = "map";
  }

  // 6) Publish
  pub_pcl_out.publish(pcl_ros_msg);
}

//------------------ Main Function ------------------//

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub_eigen_param");
  ros::NodeHandle nh("~");  // Use private namespace for parameter loading

  ROS_INFO("[livox_repub] Node started.");

  // 1) Read 4 parameters
  nh.param<bool>("global", global_coord, false);
  nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/livox/lidar");
  nh.param<std::string>("input_pose_topic",       input_pose_topic,       "/mavros/local_position/pose");
  nh.param<std::string>("output_pointcloud_topic",output_pointcloud_topic,"/pointcloud");

  ROS_INFO_STREAM("  global: " << (global_coord ? "true" : "false"));
  ROS_INFO_STREAM("  input_pointcloud_topic: " << input_pointcloud_topic);
  ROS_INFO_STREAM("  input_pose_topic: " << input_pose_topic);
  ROS_INFO_STREAM("  output_pointcloud_topic: " << output_pointcloud_topic);

  // 2) Subscribe to Pose
  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(
      input_pose_topic, 
      10, 
      PoseCallback
  );

  // 3) Subscribe to Livox custom messages
  ros::Subscriber sub_livox = nh.subscribe<livox_ros_driver::CustomMsg>(
      input_pointcloud_topic, 
      100, 
      LivoxMsgCbk
  );

  // 4) Publish the transformed point cloud
  pub_pcl_out = nh.advertise<sensor_msgs::PointCloud2>(
      output_pointcloud_topic, 
      100
  );

  ros::spin();
  return 0;
}
