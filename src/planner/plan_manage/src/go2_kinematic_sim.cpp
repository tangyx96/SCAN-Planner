#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace
{
// 简化运动学仿真中的角速度安全上限，与闭环控制器保持一致。
constexpr double kMaxVYawLimit = 1.0;

// 仿真状态、速度指令和 ROS 通信对象。
ros::Publisher odom_pub;
ros::Subscriber cmd_sub;
ros::Timer sim_timer;
tf::TransformBroadcaster *tf_broadcaster = nullptr;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double yaw = 0.0;

double vx_cmd = 0.0;
double vy_cmd = 0.0;
double vyaw_cmd = 0.0;
double vx_world = 0.0;
double vy_world = 0.0;

double max_vx = 0.8;
double max_vy = 0.5;
double max_vyaw = kMaxVYawLimit;
double cmd_timeout = 0.3;
double sim_rate = 100.0;
bool publish_tf = false;
std::string frame_id = "world";
std::string child_frame_id = "base";
std::string body_pose_topic = "/quad_0/body_pose";

ros::Time last_cmd_time;
ros::Time last_sim_time;

double clamp(double value, double min_value, double max_value)
{
  // 对输入速度进行限幅，模拟机器人底层执行器的速度边界。
  return std::max(min_value, std::min(max_value, value));
}

double normalizeAngle(double angle)
{
  // 将仿真 yaw 约束到 [-pi, pi]，防止角度随时间无限增长。
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void loadParamWithFallback(const ros::NodeHandle &nh, const std::string &private_name,
                           const std::string &fallback_name, double &value, double default_value)
{
  // 按“私有参数—全局控制器参数—代码默认值”的优先级读取仿真参数，
  // 使仿真模型能够复用闭环控制器的速度限制配置。
  if (nh.getParam(private_name, value))
    return;
  if (ros::param::get(fallback_name, value))
    return;
  value = default_value;
}

void cmdCallback(const geometry_msgs::TwistConstPtr &msg)
{
  // ==================== 1. 接收并限幅速度指令 ====================
  // cmd_vel 的线速度在机体坐标系中表达，角速度为绕 z 轴的 yaw rate。
  // 这里仅保存限幅后的指令，实际位姿积分在 simCallback 中完成。
  vx_cmd = clamp(msg->linear.x, -max_vx, max_vx);
  vy_cmd = clamp(msg->linear.y, -max_vy, max_vy);
  vyaw_cmd = clamp(msg->angular.z, -max_vyaw, max_vyaw);
  last_cmd_time = ros::Time::now();
}

void publishOdom(const ros::Time &stamp)
{
  // ==================== 2. 发布仿真状态 ====================
  // 将积分得到的位姿、世界系速度和航向封装为 nav_msgs/Odometry，供
  // SCANReplanFSM、地图模块和闭环控制器作为机器人状态反馈使用。
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);

  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = vx_world;
  odom.twist.twist.linear.y = vy_world;
  odom.twist.twist.angular.z = vyaw_cmd;
  odom_pub.publish(odom);

  if (!publish_tf || tf_broadcaster == nullptr)
    return;

  // 可选发布 world 到 base 的 TF，便于 RViz 和其他模块使用仿真坐标变换。
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = frame_id;
  tf_msg.child_frame_id = child_frame_id;
  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;
  tf_msg.transform.rotation = q;
  tf_broadcaster->sendTransform(tf_msg);
}

void simCallback(const ros::TimerEvent &)
{
  // ==================== 3. 按时间步积分简化平面运动学 ====================
  // 该模型不包含腿部动力学、接触约束或控制延迟，只根据机体速度指令
  // 更新 x、y 和 yaw，用于验证规划器与闭环速度控制器的通信链路。
  const ros::Time now = ros::Time::now();
  double dt = (now - last_sim_time).toSec();
  last_sim_time = now;
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  // 若超过 cmd_timeout 未收到新指令，则将速度清零，模拟通信看门狗。
  double vx = vx_cmd;
  double vy = vy_cmd;
  double wz = vyaw_cmd;
  if ((now - last_cmd_time).toSec() > cmd_timeout)
  {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  // 将机体系速度旋转到世界坐标系，再进行位置积分；yaw 使用角速度积分。
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_world = c * vx - s * vy;
  vy_world = s * vx + c * vy;

  x += vx_world * dt;
  y += vy_world * dt;
  yaw = normalizeAngle(yaw + wz * dt);

  publishOdom(now);
}
} // namespace

int main(int argc, char **argv)
{
  // ==================== 4. 初始化仿真节点 ====================
  // 读取初始位姿、速度上限、超时保护、发布频率和坐标系配置，
  // 随后订阅 cmd_vel、发布机身里程计并启动定时积分循环。
  ros::init(argc, argv, "go2_kinematic_sim");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::param::param("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  ros::param::param("/init_x", x, 0.0);
  ros::param::param("/init_y", y, 0.0);
  ros::param::param("/init_z", z, 0.3);
  nh.param("init_yaw", yaw, 0.0);
  loadParamWithFallback(nh, "max_vx", "/closed_loop_controller/max_vx", max_vx, 0.8);
  loadParamWithFallback(nh, "max_vy", "/closed_loop_controller/max_vy", max_vy, 0.5);
  loadParamWithFallback(nh, "max_vyaw", "/closed_loop_controller/max_vyaw", max_vyaw, kMaxVYawLimit);
  if (max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN("[Go2 kinematic sim] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  nh.param("cmd_timeout", cmd_timeout, 0.3);
  nh.param("sim_rate", sim_rate, 100.0);
  nh.param("publish_tf", publish_tf, false);
  nh.param("frame_id", frame_id, std::string("world"));
  nh.param("child_frame_id", child_frame_id, std::string("base"));

  tf::TransformBroadcaster broadcaster;
  tf_broadcaster = &broadcaster;

  odom_pub = node.advertise<nav_msgs::Odometry>(body_pose_topic, 100);
  cmd_sub = node.subscribe("cmd_vel", 20, cmdCallback, ros::TransportHints().tcpNoDelay());

  last_cmd_time = ros::Time::now();
  last_sim_time = ros::Time::now();
  sim_timer = node.createTimer(ros::Duration(1.0 / sim_rate), simCallback);

  ROS_WARN("[Go2 kinematic sim] ready.");

  ros::spin();
  return 0;
}
