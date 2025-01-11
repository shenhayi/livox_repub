#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

// Livox 自定义消息
#include "livox_ros_driver/CustomMsg.h"

// 使用了点的强度和曲率，因此这里定义一个类型
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

// ----------------------- 全局变量 -----------------------

// 是否进行全局坐标变换
bool global_coord = false;

// 用于合并多个 Livox 消息
uint64_t TO_MERGE_CNT = 1;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;

// 话题名称（由 launch 文件中指定，也可在代码中指定默认值）
std::string input_topic;
std::string output_topic;
std::string pose_topic;

// 发布点云
ros::Publisher pub_pcl_out;

// 保存当前 Pose
geometry_msgs::PoseStamped current_pose;
bool pose_received = false;  // 是否已经接收到位姿消息

// ----------------------- 回调函数与工具函数 -----------------------

/**
 * @brief 接收定位信息（PoseStamped），存储至全局变量中。
 */
void PoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  current_pose = *msg;
  pose_received = true;
}

/**
 * @brief 将点云变换到当前 Pose 对应的坐标系
 * @param[in/out] pcl_in 输入/输出点云
 */
void TransformPointCloud(PointCloudXYZI& pcl_in) {
  if (!pose_received) {
    ROS_WARN_THROTTLE(5.0, 
      "[livox_repub] No pose received yet, cannot transform pointcloud.");
    return;
  }

  // 提取平移
  double tx = current_pose.pose.position.x;
  double ty = current_pose.pose.position.y;
  double tz = current_pose.pose.position.z;

  // 提取姿态四元数
  double qx = current_pose.pose.orientation.x;
  double qy = current_pose.pose.orientation.y;
  double qz = current_pose.pose.orientation.z;
  double qw = current_pose.pose.orientation.w;

  // 构造 TF2 的变换
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(tx, ty, tz));
  tf2::Quaternion tf_q(qx, qy, qz, qw);
  transform.setRotation(tf_q);

  // 对点云进行变换
  for (auto& point : pcl_in.points) {
    tf2::Vector3 pt(point.x, point.y, point.z);
    tf2::Vector3 pt_transformed = transform * pt;
    point.x = pt_transformed.x();
    point.y = pt_transformed.y();
    point.z = pt_transformed.z();
  }
}

/**
 * @brief Livox 自定义点云回调，合并并发布标准 ROS 点云
 */
void LivoxMsgCbk(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  // 将新消息加入队列
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  // 合并多个 CustomMsg
  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;

    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      // offset_time 与 time_end 的比值可用于插入时间戳信息
      float s = livox_msg->points[i].offset_time / (float)time_end;

      // 将线号和反射强度拼成 intensity
      pt.intensity = livox_msg->points[i].line
                   + livox_msg->points[i].reflectivity / 10000.0;

      // 把 s（0~1）映射成曲率或时间戳
      pt.curvature = s * 0.1;
      pcl_in.push_back(pt);
    }
  }

  // 如果需要进行全局坐标变换，则对点云进行变换
  if (global_coord) {
    TransformPointCloud(pcl_in);
  }

  // 获取时间戳
  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  // 转成 ROS 的 PointCloud2 类型
  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "map";

  // 发布
  pub_pcl_out.publish(pcl_ros_msg);

  // 清空队列
  livox_data.clear();
}

// ----------------------- 主函数 -----------------------

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh("~");

  ROS_INFO("[livox_repub] Node started.");

  // 1. 读取参数（是否进行全局坐标变换）
  nh.param<bool>("global", global_coord, false); 
  ROS_INFO_STREAM("[livox_repub] Global transform: " 
                  << (global_coord ? "true" : "false"));

  // 2. 读取输入/输出话题名称
  nh.param<std::string>("input_topic", input_topic, "/livox/lidar");
  nh.param<std::string>("output_topic", output_topic, "/pointcloud");
  ROS_INFO_STREAM("[livox_repub] Input  topic: " << input_topic);
  ROS_INFO_STREAM("[livox_repub] Output topic: " << output_topic);

  // 3. 读取 Pose 话题名称 (默认 /mavros/local_position/pose)
  nh.param<std::string>("pose_topic", pose_topic, "/mavros/local_position/pose");
  ROS_INFO_STREAM("[livox_repub] Pose   topic: " << pose_topic);

  // 4. 订阅定位消息 PoseStamped
  ros::Subscriber sub_pose =
      nh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 1, PoseCallback);

  // 5. 订阅 Livox 点云（自定义消息）
  ros::Subscriber sub_livox_msg =
      nh.subscribe<livox_ros_driver::CustomMsg>(input_topic, 100, LivoxMsgCbk);

  // 6. 发布标准 ROS 点云
  pub_pcl_out = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 100);

  // 7. 消息循环
  ros::spin();
  return 0;
}
