#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Livox 自定义消息
#include "livox_ros_driver/CustomMsg.h"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// 发布器
ros::Publisher pub_pcl_out1;

// 用于合并多帧 Livox 消息
uint64_t TO_MERGE_CNT = 1; 
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

// 当前 Pose (假设表示“传感器在 map 坐标系下的姿态”)
geometry_msgs::PoseStamped current_pose;
bool pose_received = false;  // 是否已接收到定位信息

/**
 * @brief Pose 回调函数：订阅 /mavros/local_position/pose
 */
void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  current_pose = *pose_msg;
  pose_received = true;
}

/**
 * @brief 使用 Eigen 将点云从“传感器坐标系”变换到“世界(map)坐标系”。
 *        假设 current_pose 表示传感器在 map 坐标系下的姿态 (位姿)。
 */
void transformPointCloud(PointCloudXYZI& pcl_in) {
  if (!pose_received) {
    // 若还没接收过定位信息，可以选择直接返回或者打印警告
    ROS_WARN_THROTTLE(5.0, "[livox_repub_eigen] Pose not received yet, skipping transform.");
    return;
  }

  // 提取平移
  Eigen::Vector3d t(
      current_pose.pose.position.x,
      current_pose.pose.position.y,
      current_pose.pose.position.z
  );

  // 提取四元数 (注意顺序：Eigen 的构造函数是 (w, x, y, z))
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
    // 从“传感器系”变到“map系”
    Eigen::Vector3d v_map = T * v;
    pt.x = static_cast<float>(v_map.x());
    pt.y = static_cast<float>(v_map.y());
    pt.z = static_cast<float>(v_map.z());
  }
}

/**
 * @brief 订阅 Livox 自定义消息的回调：合并并转换成标准点云
 */
void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  // 将当前帧存起来
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) {
    return;
  }

  // 准备一个 PCL 点云容器
  pcl::PointCloud<PointType> pcl_in;

  // 合并多个 CustomMsg
  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    if (livox_msg->points.empty()) continue;
    // 用于计算相对时间
    auto time_end = livox_msg->points.back().offset_time;

    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;

      // ratio for offset_time
      float s = livox_msg->points[i].offset_time / (float)time_end;
      // line + reflectivity 拼接到 intensity
      pt.intensity = livox_msg->points[i].line 
                   + livox_msg->points[i].reflectivity / 10000.0f;
      pt.curvature = s * 0.1f;

      pcl_in.push_back(pt);
    }
  }

  // 清空缓存
  livox_data.clear();

  // ============= 坐标变换：传感器系 -> map系 =============
  transformPointCloud(pcl_in);

  // 获取时间戳
  unsigned long timebase_ns = livox_msg_in->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  // 转换为 ROS 的点云消息
  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);

  // 既然已经变换到 map 坐标系，就设置 frame_id 为 "map"
  pcl_ros_msg.header.frame_id = "map";

  // 发布
  pub_pcl_out1.publish(pcl_ros_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub_eigen");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub_eigen");

  // 1) 订阅 Livox 自定义话题
  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 
      100, 
      LivoxMsgCbk1
  );

  // 2) 订阅定位话题 (PoseStamped)，假设: “传感器在 map 坐标系下的姿态”
  ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose",
      10,
      PoseCallback
  );

  // 3) 发布转换后的标准点云
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud", 100);

  ros::spin();
  return 0;
}
