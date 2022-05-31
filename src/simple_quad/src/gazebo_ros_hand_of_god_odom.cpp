// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns (gazebo_ros_diff_drive)
// Copyright (c) 2013, Open Source Robotics Foundation (gazebo_ros_hand_of_god)
// Copyright (c) 2022, Ricardo de Azambuja
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/*
 * \file  gazebo_ros_hand_of_god_odom.cpp
 *
 * \brief A new "hand-of-god" plugin with added odometry output.
 *  Odometry output (nav_msgs::msg::Odometry) was stolen from 
 *  gazebo_ros_diff_drive plugin.
 *
 */


#include <gazebo/common/Time.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <simple_quad/gazebo_ros_hand_of_god_odom.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosHandOfGodOdomPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a Pose command is received.
  /// \param[in] _msg Pose command message.
  void OnCmdPos(const geometry_msgs::msg::Pose::SharedPtr _msg);

    /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Subscriber to command poses
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pos_sub_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// To broadcast TFs
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Update odometry according to world
  void UpdateOdometryWorld();

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// Publish base_footprint transforms
  /// \param[in] _current_time Current simulation time
  void PublishFootprintTf(const gazebo::common::Time & _current_time);

  /// Publish odometry messages
  /// \param[in] _current_time Current simulation time
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Pointer to link.
  gazebo::physics::LinkPtr link_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Last update time.
  gazebo::common::Time last_main_update_time_;

  /// Keep encoder data.
  geometry_msgs::msg::Pose recv_pose_;

  /// Linear velocity in X received on command (m/s).
  tf2::Vector3 recv_linear_vel_{0.0,0.0,0.0};
  tf2::Vector3 target_linear_vel_{0.0,0.0,0.0};


  /// Angular velocity in Z received on command (rad/s).
  double target_rot_{0.0};

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// Covariance in odometry
  double covariance_[12];

  /// Bias in odometry
  double bias_[12];

  /// Update period in seconds.
  double update_period_;

  /// frame ID
  std::string frame_;

  /// Applied force and torque gains
  double kl_, ka_, cl_, ca_;

  double mass_;

  double max_acc_;

  bool follow_recv_pose_{false};
};

GazeboRosHandOfGodOdom::GazeboRosHandOfGodOdom()
: impl_(std::make_unique<GazeboRosHandOfGodOdomPrivate>())
{
}

GazeboRosHandOfGodOdom::~GazeboRosHandOfGodOdom()
{
}

void GazeboRosHandOfGodOdom::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;
  
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf, _model);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  auto pose = impl_->model_->WorldPose();
  impl_->recv_pose_.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  impl_->recv_pose_.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
    RCLCPP_WARN(impl_->ros_node_->get_logger(), "Update period set to ZERO!");
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  // From GazeboRosHandOfGod
  impl_->frame_ = _sdf->Get<std::string>("frame_id", "world").first;

  impl_->kl_ = _sdf->Get<double>("kl", 200).first;
  impl_->ka_ = _sdf->Get<double>("ka", 200).first;

  if (_sdf->HasElement("link_name")) {
    auto link_name = _sdf->Get<std::string>("link_name");
    impl_->link_ = _model->GetLink(link_name);
    if (!impl_->link_) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(), "Link [%s] not found. Aborting", link_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
  } else {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Please specify <link_name>. Aborting");
    impl_->ros_node_.reset();
    return;
  }

  impl_->link_->SetGravityMode(false);

  impl_->mass_ = impl_->link_->GetInertial()->Mass();
  impl_->cl_ = 2.0 * sqrt(impl_->kl_ * impl_->mass_);
  impl_->ca_ = 2.0 * sqrt(impl_->ka_ * impl_->link_->GetInertial()->IXX());

  // Subscribe to pose
  impl_->cmd_pos_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Pose>(
    "cmd_pos", qos.get_subscription_qos("cmd_pos", rclcpp::QoS(1)),
    std::bind(&GazeboRosHandOfGodOdomPrivate::OnCmdPos, impl_.get(), std::placeholders::_1));
  
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_pos_sub_->get_topic_name());

  // Subscribe to twist
  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosHandOfGodOdomPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]", 
    impl_->cmd_vel_sub_->get_topic_name());

  // To publish TFs ourselves
  impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_link").first;

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_odom_tf_) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
      impl_->robot_base_frame_.c_str());
  }

  impl_->covariance_[0] =  _sdf->Get<double>("covariance_x",     0.00001).first;
  impl_->covariance_[1] =  _sdf->Get<double>("covariance_y",     0.00001).first;
  impl_->covariance_[2] =  _sdf->Get<double>("covariance_z",     0.00001).first;
  impl_->covariance_[3] =  _sdf->Get<double>("covariance_roll",  0.001).first;
  impl_->covariance_[4] =  _sdf->Get<double>("covariance_pitch", 0.001).first;
  impl_->covariance_[5] =  _sdf->Get<double>("covariance_yaw",   0.001).first;
  impl_->covariance_[6] =  _sdf->Get<double>("covariance_vx",     0.00001).first;
  impl_->covariance_[7] =  _sdf->Get<double>("covariance_vy",     0.00001).first;
  impl_->covariance_[8] =  _sdf->Get<double>("covariance_vz",     0.00001).first;
  impl_->covariance_[9] =  _sdf->Get<double>("covariance_vroll",  0.001).first;
  impl_->covariance_[10] = _sdf->Get<double>("covariance_vpitch", 0.001).first;
  impl_->covariance_[11] = _sdf->Get<double>("covariance_vyaw",   0.001).first;

  impl_->bias_[0] =  _sdf->Get<double>("bias_x",     0.00001).first;
  impl_->bias_[1] =  _sdf->Get<double>("bias_y",     0.00001).first;
  impl_->bias_[2] =  _sdf->Get<double>("bias_z",     0.00001).first;
  impl_->bias_[3] =  _sdf->Get<double>("bias_roll",  0.001).first;
  impl_->bias_[4] =  _sdf->Get<double>("bias_pitch", 0.001).first;
  impl_->bias_[5] =  _sdf->Get<double>("bias_yaw",   0.001).first;
  impl_->bias_[6] =  _sdf->Get<double>("bias_vx",     0.00001).first;
  impl_->bias_[7] =  _sdf->Get<double>("bias_vy",     0.00001).first;
  impl_->bias_[8] =  _sdf->Get<double>("bias_vz",     0.00001).first;
  impl_->bias_[9] =  _sdf->Get<double>("bias_vroll",  0.001).first;
  impl_->bias_[10] = _sdf->Get<double>("bias_vpitch", 0.001).first;
  impl_->bias_[11] = _sdf->Get<double>("bias_vyaw",   0.001).first;

  // Set covariance
  impl_->odom_.pose.covariance[0] =  impl_->covariance_[0];
  impl_->odom_.pose.covariance[7] =  impl_->covariance_[1];
  impl_->odom_.pose.covariance[14] = impl_->covariance_[2];
  impl_->odom_.pose.covariance[21] = impl_->covariance_[3];
  impl_->odom_.pose.covariance[28] = impl_->covariance_[4];
  impl_->odom_.pose.covariance[35] = impl_->covariance_[5];

  impl_->odom_.twist.covariance[0] =  impl_->covariance_[6];
  impl_->odom_.twist.covariance[7] =  impl_->covariance_[7];
  impl_->odom_.twist.covariance[14] = impl_->covariance_[8];
  impl_->odom_.twist.covariance[21] = impl_->covariance_[9];
  impl_->odom_.twist.covariance[28] = impl_->covariance_[10];
  impl_->odom_.twist.covariance[35] = impl_->covariance_[11];


  impl_->max_acc_ = _sdf->Get<double>("max_acc", 10).first;
  
  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosHandOfGodOdomPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosHandOfGodOdomPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("Get pose command");
#endif
  
  // Time delta
  double dt = (_info.simTime - last_main_update_time_).Double();
  last_main_update_time_ = _info.simTime;

  ignition::math::Pose3d hog_desired;
  ignition::math::Pose3d curr_pose = link_->DirtyPose();

  {
    std::lock_guard<std::mutex> pose_lock(lock_);

    // Modify recv_pose_ according to the received velocities.
    tf2::Quaternion q_rot, q_new; // https://docs.ros.org/en/galactic/Tutorials/Tf2/Quaternion-Fundamentals.html
    tf2::fromMsg(recv_pose_.orientation, q_new);
    tf2::Matrix3x3 m(q_new);

    auto target_acc = (recv_linear_vel_-target_linear_vel_)/dt;
    auto target_acc_length = target_acc.length();
    if ( target_acc_length > max_acc_) {
      target_linear_vel_ += (target_acc/target_acc_length)*max_acc_*dt;
    } else {
      target_linear_vel_ = recv_linear_vel_;
    }

    // Rotate the target_linear_vel_ vector to align it to recv_pose_.orientation
    // because the commands are in the drone frame
    auto target_linear_vel_cmd_rotated = m*(target_linear_vel_*dt*kl_);
    auto target_rot_cmd = target_rot_*dt;

    if (target_linear_vel_.length()>0 && !follow_recv_pose_){
      recv_pose_.position.x = curr_pose.Pos().X();
      recv_pose_.position.y = curr_pose.Pos().Y();
      recv_pose_.position.z = curr_pose.Pos().Z();
    }

    if (abs(target_rot_)>0 && !follow_recv_pose_){
      recv_pose_.orientation.x = curr_pose.Rot().X();
      recv_pose_.orientation.y = curr_pose.Rot().Y();
      recv_pose_.orientation.z = curr_pose.Rot().Z();
      recv_pose_.orientation.w = curr_pose.Rot().W();
    }

    recv_pose_.position.x += target_linear_vel_cmd_rotated.getX();
    recv_pose_.position.y += target_linear_vel_cmd_rotated.getY();
    recv_pose_.position.z += target_linear_vel_cmd_rotated.getZ();

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    q_rot.setRPY(0.0, 0.0, target_rot_cmd);
    q_new = q_rot * q_new;
    m.setRotation(q_new);
    m.getRPY(roll, pitch, yaw);
    q_new.setRPY(0.0, 0.0, yaw); // The drone can only hover
    q_new.normalize();
    tf2::convert(q_new, recv_pose_.orientation); // recv_pose_.orientation = tf2::toMsg(q_new);
  
    hog_desired = gazebo_ros::Convert<ignition::math::Pose3d>(recv_pose_);
  }

  /// Track recv_pose_
  
  // Current velocity
  ignition::math::Vector3d world_linear_vel = link_->WorldLinearVel();

  // Relative transform from actual to desired pose
  ignition::math::Vector3d relative_angular_vel = link_->RelativeAngularVel();

  ignition::math::Vector3d err_pos = hog_desired.Pos() - curr_pose.Pos();
  ignition::math::Vector3d force = (kl_ * err_pos - cl_ * world_linear_vel);

  // Get exponential coordinates for rotation
  ignition::math::Quaterniond err_rot = (ignition::math::Matrix4d(curr_pose.Rot()).Inverse() *
    ignition::math::Matrix4d(hog_desired.Rot())).Rotation();

  ignition::math::Vector3d err_vec(err_rot.Log().X(), err_rot.Log().Y(), err_rot.Log().Z());
  ignition::math::Vector3d torque = (ka_ * err_vec - ca_ * relative_angular_vel);

  link_->AddForce(force);
  link_->AddRelativeTorque(torque);

  PublishFootprintTf(_info.simTime);
  

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("UpdateOdometryWorld");
#endif
  // Update odom message if using ground truth
  UpdateOdometryWorld();

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
  if (publish_odom_) {
    PublishOdometryMsg(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
  if (publish_odom_tf_) {
    PublishOdometryTf(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

  last_update_time_ = _info.simTime;
}

void GazeboRosHandOfGodOdomPrivate::OnCmdPos(const geometry_msgs::msg::Pose::SharedPtr _msg)
{
  std::lock_guard<std::mutex> pose_lock(lock_);
  recv_pose_.position = _msg->position;
  recv_pose_.orientation = _msg->orientation;
  follow_recv_pose_ = true;
}

void GazeboRosHandOfGodOdomPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  recv_linear_vel_.setX(_msg->linear.x);
  recv_linear_vel_.setY(_msg->linear.y);
  recv_linear_vel_.setZ(_msg->linear.z);
  target_rot_ = _msg->angular.z;
  follow_recv_pose_ = false;
}

void GazeboRosHandOfGodOdomPrivate::UpdateOdometryWorld()
{
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  odom_.pose.pose.position.x += ignition::math::Rand::DblNormal(bias_[0], covariance_[0]);
  odom_.pose.pose.position.y += ignition::math::Rand::DblNormal(bias_[1], covariance_[1]);
  odom_.pose.pose.position.z += ignition::math::Rand::DblNormal(bias_[2], covariance_[2]);

  ignition::math::Quaternion q_noise(ignition::math::Rand::DblNormal(bias_[3], covariance_[3]), 
                                     ignition::math::Rand::DblNormal(bias_[4], covariance_[4]),
                                     ignition::math::Rand::DblNormal(bias_[5], covariance_[5]));
  
  odom_.pose.pose.orientation.x += q_noise.X();
  odom_.pose.pose.orientation.y += q_noise.Y();
  odom_.pose.pose.orientation.z += q_noise.Z();
  odom_.pose.pose.orientation.w += q_noise.W();

  // Get velocity in odom frame
  auto linear = model_->RelativeLinearVel();
  odom_.twist.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(linear);
  odom_.twist.twist.linear.x += ignition::math::Rand::DblNormal(bias_[6], covariance_[6]);
  odom_.twist.twist.linear.y += ignition::math::Rand::DblNormal(bias_[7], covariance_[7]);
  odom_.twist.twist.linear.z += ignition::math::Rand::DblNormal(bias_[8], covariance_[8]);

  
  auto angular = model_->RelativeAngularVel();
  odom_.twist.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(angular);
  odom_.twist.twist.angular.x += ignition::math::Rand::DblNormal(bias_[9], covariance_[9]);
  odom_.twist.twist.angular.y += ignition::math::Rand::DblNormal(bias_[10], covariance_[10]);
  odom_.twist.twist.angular.z += ignition::math::Rand::DblNormal(bias_[11], covariance_[11]);
}

void GazeboRosHandOfGodOdomPrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosHandOfGodOdomPrivate::PublishFootprintTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = robot_base_frame_;
  msg.child_frame_id = "base_footprint";
  msg.transform.translation.z = -(gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position)).z;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosHandOfGodOdomPrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosHandOfGodOdom)
}  // namespace gazebo_plugins
