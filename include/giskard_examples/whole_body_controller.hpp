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

#ifndef __GISKARD_WHOLE_BODY_CONTROLLER_HPP__
#define __GISKARD_WHOLE_BODY_CONTROLLER_HPP__

#include <ros/ros.h>
#include <giskard/giskard.hpp>
#include <sensor_msgs/JointState.h>
#include <giskard_msgs/WholeBodyCommand.h>
#include <giskard_msgs/ControllerFeedback.h>
#include <giskard_msgs/SemanticFloat64Array.h>

namespace giskard_examples
{
  class WholeBodyControllerParams
  {
    public:
      std::string frame_id, l_fk_name, r_fk_name;
      std::vector< std::string > joint_names, l_arm_names, r_arm_names;
      std::map<std::string, std::string> controller_descriptions;
      std::set< std::string > controller_types;
      int nWSR;
  };

  class ControllerContext
  {
    private:
      giskard::QPController controller_;
      Eigen::VectorXd state_;
      giskard_msgs::ControllerFeedback feedback_;
      giskard_msgs::SemanticFloat64Array vel_command_;

    public:
      void set_controller(const giskard::QPController& controller);

      void set_joint_state(const sensor_msgs::JointState& msg);

      void set_command(const giskard_msgs::WholeBodyCommand& command);

      bool update(const sensor_msgs::JointState& msg, int nWSR);

      void start_controller(const giskard_msgs::WholeBodyCommand& command,
          const WholeBodyControllerParams& params,
          const sensor_msgs::JointState& msg, 
          const std::string& name);

      const giskard::QPController& get_controller() const;

      const giskard_msgs::WholeBodyCommand& get_command() const;

      const giskard_msgs::ControllerFeedback& get_feedback() const;

      const giskard_msgs::SemanticFloat64Array& get_vel_command() const;
  };

  enum class WholeBodyControllerState { constructed, started, running };

  class WholeBodyController
  {
    public:
      WholeBodyController(const ros::NodeHandle& nh); 
      ~WholeBodyController();

      // TODO: think about joining init & start
      void init(const WholeBodyControllerParams& params);

      void start(const sensor_msgs::JointState& msg);

      void update(const sensor_msgs::JointState& msg);

      void set_command(const giskard_msgs::WholeBodyCommand& msg);

      // TODO: get rid of these callbacks
      void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

      void command_callback(const giskard_msgs::WholeBodyCommand::ConstPtr& msg);

      const WholeBodyControllerState& state() const;

    private:
      // TODO: separate into two classes to allow unit testing
      ros::NodeHandle nh_;
      ros::Publisher velocity_pub_, feedback_pub_;
      ros::Subscriber goal_sub_, joint_state_sub_;

      std::map< std::string, ControllerContext > contexts_;
      std::string current_controller_;
      WholeBodyControllerParams parameters_;
      WholeBodyControllerState state_;
      sensor_msgs::JointState last_joint_state_;

      // INTERNAL HELPER FUNCTIONS
      // TODO: check whether these could move somewhere else

      ControllerContext& get_current_context();

      ControllerContext& get_context(const std::string& controller);

      giskard_msgs::WholeBodyCommand complete_command(const giskard_msgs::WholeBodyCommand& new_command,
          const giskard_msgs::WholeBodyCommand& current_command);

      std::string infer_controller(const giskard_msgs::WholeBodyCommand& msg);


      void init_and_start_yaml_controller(const giskard_msgs::WholeBodyCommand& msg);

      void init_controller_contexts();

      KDL::Frame eval_fk(const std::string& fk_name, const sensor_msgs::JointState& msg);
  };
}

#endif // __GISKARD_WHOLE_BODY_CONTROLLER__HPP
