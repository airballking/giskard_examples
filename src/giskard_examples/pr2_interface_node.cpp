/*
* Copyright (C) 2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
*
*
* This file is part of giskard_examples.
*
* giskard_examples is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2 
* of the License, or (at your option) any later version.  
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <giskard_msgs/SemanticFloat64Array.h>
#include <exception>
#include <giskard_examples/ros_utils.hpp>

namespace giskard_examples
{
  enum class NodeState { constructed, started, running };

  size_t find_index(const sensor_msgs::JointState& msg, const std::string& name)
  {
     for (size_t i=0; i < msg.name.size(); ++i)
       if (name.find(msg.name[i]) == 0)
         return i;

     throw std::runtime_error("Could not find joint-state with name '" + name + "'.");
  }

  size_t find_index(const giskard_msgs::SemanticFloat64Array& msg, const std::string& name)
  {
     for (size_t i=0; i < msg.data.size(); ++i)
       if (name.find(msg.data[i].semantics) == 0)
         return i;

     throw std::runtime_error("Could not find semantic float64 name '" + name + "'.");
  }

  class PR2InterfaceNode
  {
    public:
      PR2InterfaceNode(const ros::NodeHandle& nh) : 
          nh_(nh) , state_(NodeState::constructed)
      {}
      ~PR2InterfaceNode() {}

      void start() 
      {
        if(state_ == NodeState::constructed)
        {
          pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("position_cmd", 1);
          read_parameters();
          init_command();
          vel_command_sub_ = nh_.subscribe("velocity_cmd", 1, 
              &PR2InterfaceNode::vel_command_callback, this);
          joint_states_sub_ = nh_.subscribe("joint_states", 1, 
              &PR2InterfaceNode::joint_state_callback, this);
          state_ = NodeState::started;
        }
      }

    private:
      ros::NodeHandle nh_;
      ros::Subscriber joint_states_sub_, vel_command_sub_;
      ros::Publisher pub_;

      trajectory_msgs::JointTrajectory command_;
      giskard_msgs::SemanticFloat64Array last_vel_command_;
      std::vector<std::string> joint_names_;
      double period_;
      NodeState state_;

      void read_parameters()
      {
        joint_names_ = readParam< std::vector<std::string> >(nh_, "joint_names");
        period_ = readParam<double>(nh_, "period");
      }

      void init_command()
      {
        command_.joint_names = joint_names_;
        command_.points.resize(1);
        command_.points[0].positions.resize(joint_names_.size());
        command_.points[0].time_from_start = ros::Duration(period_);
      }

      void vel_command_callback(const giskard_msgs::SemanticFloat64Array::ConstPtr& msg)
      {
        last_vel_command_ = *msg;
        state_ = NodeState::running;
      }

      void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        if (state_ == NodeState::running)
        {
          command_.header.stamp = msg->header.stamp;
          for (size_t i=0; i < joint_names_.size(); ++i)
          {
            size_t joint_index = find_index(*msg, joint_names_[i]);
            size_t cmd_index = find_index(last_vel_command_, joint_names_[i]);

            command_.points[0].positions[i] = msg->position[joint_index] +
              period_ * last_vel_command_.data[cmd_index].value;
          }

//          command_.points[0].positions[6] += period_ * 0.1;

          pub_.publish(command_);
        }
      }
   };
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_interface_node");
  ros::NodeHandle nh("~");

  giskard_examples::PR2InterfaceNode pr2(nh);

  try
  {
    pr2.start();
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
