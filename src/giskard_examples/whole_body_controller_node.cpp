/*
* Copyright (C) 2015, 2016 Jannik Buckelo <jannikbu@cs.uni-bremen.de>,
* Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#include <giskard_examples/whole_body_controller.hpp>

namespace giskard_examples
{
  class WholeBodyControllerNode
  {
    public:
      WholeBodyControllerNode(const ros::NodeHandle& nh) : 
        nh_(nh) {}
      ~WholeBodyControllerNode() {}

      void start()
      {
        wbc_.init(read_params(nh_));

        feedback_pub_ = nh_.advertise<giskard_msgs::ControllerFeedback>("feedback", 1, true);
        velocity_pub_ = nh_.advertise<giskard_msgs::SemanticFloat64Array>("velocity_cmd", 1);
        joint_state_sub_ = nh_.subscribe("joint_states", 1, &WholeBodyControllerNode::joint_state_callback, this,
          ros::TransportHints().tcpNoDelay());

      }

    private:

      void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
      {
        if (wbc_.state() == WholeBodyControllerState::started)
        {
          wbc_.start(*msg);
          goal_sub_ = nh_.subscribe("goal", 1, &WholeBodyControllerNode::command_callback, this);
        }

        wbc_.update(*msg);
        velocity_pub_.publish(wbc_.vel_command());
        feedback_pub_.publish(wbc_.feedback());
      }

      void command_callback(const giskard_msgs::WholeBodyCommand::ConstPtr& msg)
      {
        wbc_.set_command(*msg);
      }

      ros::NodeHandle nh_;
      ros::Publisher velocity_pub_, feedback_pub_;
      ros::Subscriber goal_sub_, joint_state_sub_;

      WholeBodyController wbc_;
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "whole_body_controller");
  ros::NodeHandle nh("~");

  using namespace giskard_examples;

  WholeBodyControllerNode controller(nh);
  try
  {
    controller.start();
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
