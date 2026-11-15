#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

// 对角速度设置统一的安全上限，防止参数配置导致过大的原地旋转指令。
constexpr double kMaxVYawLimit = 1.0;

// ROS 通信对象：分别用于发布速度指令、执行冻结状态，并接收轨迹和里程计。
ros::Publisher cmd_vel_pub;
ros::Publisher execution_frozen_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Timer cmd_timer;

bool receive_traj = false;
bool have_odom = false;
// traj[0]、traj[1]、traj[2] 分别保存位置、速度和加速度 B 样条。
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double exec_time = 0.0;
ros::Time last_update_time;

double time_forward;
double heading_error_threshold;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
std::string body_pose_topic;

bool loadRequiredParam(const ros::NodeHandle &nh, const std::string &name, double &value)
{
  // 读取必须存在的私有参数，缺少关键参数时拒绝启动控制器。
  if (nh.getParam(name, value))
    return true;

  ROS_ERROR_STREAM("[closed_loop_controller] missing required private parameter ~" << name);
  return false;
}

bool loadParams(const ros::NodeHandle &nh)
{
  // 集中读取控制器参数，并对最大 yaw 速度施加额外的安全限幅。
  bool ok = true;
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  ok &= loadRequiredParam(nh, "time_forward", time_forward);
  ok &= loadRequiredParam(nh, "heading_error_threshold", heading_error_threshold);
  ok &= loadRequiredParam(nh, "kp_pos", kp_pos);
  ok &= loadRequiredParam(nh, "kp_yaw", kp_yaw);
  ok &= loadRequiredParam(nh, "max_vx", max_vx);
  ok &= loadRequiredParam(nh, "max_vy", max_vy);
  ok &= loadRequiredParam(nh, "max_vyaw", max_vyaw);
  ok &= loadRequiredParam(nh, "finish_dist", finish_dist);
  if (ok && max_vyaw > kMaxVYawLimit)
  {
    ROS_WARN("[closed_loop_controller] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  return ok;
}

double normalizeAngle(double angle)
{
  // 将角度归一化到 [-pi, pi]，避免航向误差跨越边界时产生跳变。
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  // 将标量控制量限制在给定上下界内。
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  // 按向量范数进行限幅，保持速度方向不变，避免分别限幅造成方向偏差。
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
  // 用前视点与期望位置的水平连线估计期望航向；局部位移过小时退化为
  // 轨迹速度方向，仍不足时保持当前里程计航向。
  const double t_look = std::min(traj_duration, t_cur + time_forward);
  Eigen::Vector3d dir = traj[0].evaluateDeBoorT(t_look) - pos_des;

  if (dir.head<2>().squaredNorm() < 1e-4)
  {
    Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_cur);
    dir = vel;
  }

  if (dir.head<2>().squaredNorm() < 1e-4)
    return odom_yaw;

  return std::atan2(dir(1), dir(0));
}

void publishStop(double vyaw = 0.0)
{
  // 发布零平移速度，并保留经过限幅的角速度，用于原地转向或安全停止。
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  // 通知规划状态机是否暂停局部轨迹的时间推进。
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  // 将规划器发布的控制点和节点向量重建为位置、速度和加速度 B 样条。
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  // 将 ROS 消息中的控制点和节点向量恢复为 UniformBspline，随后由样条求导
  // 获得速度和加速度轨迹。新轨迹到达后从执行时间零点重新开始跟踪。
  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  receive_traj = true;

  ROS_WARN("[closed_loop_controller] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // 保存当前机身位置和 yaw，作为位置误差反馈及机体系速度变换的测量值。
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  odom_yaw = tf::getYaw(msg->pose.pose.orientation);
  have_odom = true;
}

void cmdCallback(const ros::TimerEvent &)
{
  // 按当前执行时间连续查询轨迹状态，并向下游控制接口发布期望指令。
  if (!receive_traj || !have_odom)
  {
    // 在轨迹或里程计尚未准备好时不推进执行时间，并发布安全停止指令。
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update_time).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  // 先在当前执行时刻计算期望状态和航向误差。若航向偏差过大，
  // 控制器只允许原地转向，不让轨迹时间继续前进。
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);

  const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const double vyaw_cmd = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw);

  if (std::abs(yaw_err) > heading_error_threshold)
  {
    publishExecutionFrozen(true);
    publishStop(vyaw_cmd);
    last_update_time = now; // freeze exec_time while rotating in place
    return;
  }

  publishExecutionFrozen(false);
  // 航向满足要求后推进执行时间，并重新查询与实际控制周期对应的期望状态。
  exec_time = std::min(traj_duration, exec_time + dt);
  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  Eigen::Vector2d pos_err(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1));
  Eigen::Vector2d vel_ff(vel_des(0), vel_des(1));
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, std::max(max_vx, max_vy));

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (exec_time >= traj_duration && pos_err.norm() < finish_dist)
    cmd = geometry_msgs::Twist();

  cmd_vel_pub.publish(cmd);
}
} // namespace

int main(int argc, char **argv)
{
  // 初始化 ROS 节点、读取控制参数、建立通信接口，并以 100 Hz 运行控制回调。
  ros::init(argc, argv, "closed_loop_controller");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  if (!loadParams(nh))
    return 1;

  bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  execution_frozen_pub = node.advertise<std_msgs::Bool>("planning/go2_execution_frozen", 10);
  cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  last_update_time = ros::Time::now();
  ROS_WARN("[closed_loop_controller] ready.");

  ros::spin();
  return 0;
}
