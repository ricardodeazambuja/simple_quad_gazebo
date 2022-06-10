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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_DIFF_DRIVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_DIFF_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosSimpleQuadPrivate;

/// Drives a floating object around based on the location of a TF frame, with odometry output.

/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_simple_quad" filename="libgazebo_ros_simple_quad.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/simple_quad</namespace>

      </ros>

      <!-- This is required -->
      <link_name>link</link_name>

      <!-- Defaults to world -->
      <!-- The plugin expects TF `frame_id` + "_desired" -->
      <frame_id>link</frame_id>

      <!-- Set force and torque gains -->
      <ka>200</ka>
      <kl>200</kl>

      <!-- Max acceleration -->
      <max_acc>10</max_acc>

      <fake_pitch_roll>true</fake_pitch_roll>

      <!-- Odometry output -->
      <update_rate>50</update_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>

      <cmd_vel_topic>/cmd_vel</cmd_vel_topic>
      <cmd_pos_topic>/cmd_pos</cmd_pos_topic>

      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>

      <bias_x>0.00001</bias_x>
      <bias_y>0.00001</bias_y>
      <bias_z>0.00001</bias_z>
      <covariance_x>0.01</covariance_x>
      <covariance_y>0.01</covariance_y>
      <covariance_z>0.01</covariance_z>
      <bias_vx>0.00001</bias_vx>
      <bias_vy>0.00001</bias_vy>
      <bias_vz>0.00001</bias_vz>
      <covariance_vx>0.01</covariance_vx>
      <covariance_vy>0.01</covariance_vy>
      <covariance_vz>0.01</covariance_vz>
      <bias_roll>0.001</bias_roll>
      <bias_pitch>0.001</bias_pitch>
      <bias_yaw>0.001</bias_yaw>
      <covariance_roll>0.01</covariance_roll>
      <covariance_pitch>0.01</covariance_pitch>
      <covariance_yaw>0.01</covariance_yaw>
      <bias_vroll>0.001</bias_vroll>
      <bias_vpitch>0.001</bias_vpitch>
      <bias_vyaw>0.001</bias_vyaw>
      <covariance_vroll>0.01</covariance_vroll>
      <covariance_vpitch>0.01</covariance_vpitch>
      <covariance_vyaw>0.01</covariance_vyaw>
      
    </plugin>
  \endcode
*/
class GazeboRosSimpleQuad : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosSimpleQuad();

  /// Destructor
  ~GazeboRosSimpleQuad();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosSimpleQuadPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_DIFF_DRIVE_HPP_
