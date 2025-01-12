#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Livox 自定义消息
#include "livox_ros_driver/CustomMsg.h"

// 使用 pcl::PointXYZINormal 来保留 intensity/curvature 信息
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

//------------------ 全局变量 ------------------//

// 是否进行全局坐标变换（可通过参数 global 决定）
bool global_coord = false;

// Pose (假设表示 "传感器在 map 坐标系下" 的位姿)
geometry_msgs::PoseStamped current_pose;
bool pose_received = false;  

// 用于合并多帧
uint64_t TO_MERGE_CNT = 1; 
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

// 话题名称，由参数加载
std::string input_pointcloud_topic;
std::string input_pose_topic;
std::string output_pointcloud_topic;

// 发布器
ros::Publisher pub_pcl_out;

/**
 * @brief Pose 回调函数: 订阅 input_pose_topic
 */
void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  current_pose = *pose_msg;
  pose_received = true;
}

/**
 * @brief 将点云从“传感器系”变换到“map系” (Eigen)
 * @param[in,out] pcl_in  需变换的点云（原地修改）
 */
void transformPointCloud(PointCloudXYZI& pcl_in) {
  if (!pose_received) {
    ROS_WARN_THROTTLE(5.0, 
      "[livox_repub_eigen_param] Pose not received yet, skip transform.");
    return;
  }

  // 提取平移
  Eigen::Vector3d t(
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      current_pose.pose.position.z
  );

  // 提取四元数 (注意顺序: Eigen 构造是 (w, x, y, z))
  Eigen::Quaterniond q(
      current_pose.pose.orientation.w,
      current_pose.pose.orientation.x,
      current_pose.pose.orientation.y,
      current_pose.pose.orientation.z
  );

  // 构造变换矩阵 T_sensor_to_map
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translation() = t;
  T.linear() = q.normalized().toRotationMatrix();

  // 遍历点云，做坐标变换
  for (auto& pt : pcl_in.points) {
    Eigen::Vector3d v(pt.x, pt.y, pt.z);
    Eigen::Vector3d v_map = T * v; 
    pt.x = static_cast<float>(v_map.x());
    pt.y = static_cast<float>(v_map.y());
    pt.z = static_cast<float>(v_map.z());
  }
}

/**
 * @brief Livox 自定义点云回调函数
 */
void LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  // 将当前帧加入缓存
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) {
    return;
  }

  // 1) 创建 PCL 点云容器
  PointCloudXYZI pcl_in;

  // 2) 合并多帧
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

  // 3) 是否进行全局坐标变换
  if (global_coord) {
    // 传感器 -> map
    transformPointCloud(pcl_in);
  }

  // 4) 获取时间戳
  unsigned long timebase_ns = livox_msg_in->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  // 5) 转换为 ROS 标准点云
  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);

  // 如果全局坐标变换已执行，就写 "map"；否则可写 "livox" 或其他
  // 这里我们统一用 "map"，表示最终都是 map 系
  if(global_coord) {
    pcl_ros_msg.header.frame_id = "map";
  } else {
    pcl_ros_msg.header.frame_id = "livox";
  }

  // 6) 发布
  pub_pcl_out.publish(pcl_ros_msg);
}

//------------------ 主函数 ------------------//

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub_eigen_param");
  ros::NodeHandle nh("~");  // 使用私有命名空间，方便读取参数

  ROS_INFO("[livox_repub] Node started.");

  // 1) 读取 4 个参数
  nh.param<bool>("global", global_coord, false);
  nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/livox/lidar");
  nh.param<std::string>("input_pose_topic",       input_pose_topic,       "/mavros/local_position/pose");
  nh.param<std::string>("output_pointcloud_topic",output_pointcloud_topic,"/pointcloud");

  ROS_INFO_STREAM("  global: " << (global_coord ? "true" : "false"));
  ROS_INFO_STREAM("  input_pointcloud_topic: " << input_pointcloud_topic);
  ROS_INFO_STREAM("  input_pose_topic: " << input_pose_topic);
  ROS_INFO_STREAM("  output_pointcloud_topic: " << output_pointcloud_topic);

  // 2) 订阅 Pose
  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(
      input_pose_topic, 
      10, 
      PoseCallback
  );

  // 3) 订阅 Livox 自定义消息
  ros::Subscriber sub_livox = nh.subscribe<livox_ros_driver::CustomMsg>(
      input_pointcloud_topic, 
      100, 
      LivoxMsgCbk
  );

  // 4) 发布转换后的点云
  pub_pcl_out = nh.advertise<sensor_msgs::PointCloud2>(
      output_pointcloud_topic, 
      100
  );

  ros::spin();
  return 0;
}
