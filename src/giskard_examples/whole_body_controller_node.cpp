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
#include <giskard_examples/ros_utils.hpp>

namespace giskard_examples
{
  WholeBodyControllerParams read_params(const ros::NodeHandle& nh)
  {
    WholeBodyControllerParams params;
    params.nWSR = readParam<int>(nh, "nWSR");
    // TODO: extract joint_names from controller description
    params.joint_names = readParam< std::vector<std::string> >(nh, "joint_names");
    // TODO: harmonize with bodypart notation from other nodes?
    params.l_arm_names = readParam< std::vector<std::string> >(nh, "l_arm_names");
    params.r_arm_names = readParam< std::vector<std::string> >(nh, "r_arm_names");
    params.frame_id = readParam< std::string >(nh, "frame_id");
    params.l_fk_name = readParam< std::string >(nh, "l_fk_name");
    params.r_fk_name = readParam< std::string >(nh, "r_fk_name");
    params.controller_types = {"cart_cart", "joint_cart", "cart_joint", "joint_joint", "yaml"};

    params.controller_descriptions = 
      readParam< std::map<std::string, std::string> >(nh, "controller_descriptions");
    for (std::set<std::string>::const_iterator it=params.controller_types.begin();
         it!=params.controller_types.end(); ++it)
      if(it->compare("yaml") != 0 && 
          params.controller_descriptions.find(*it) == params.controller_descriptions.end())
        throw std::runtime_error("Could not find controller description for '" + *it + "'.");

    return params;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "whole_body_controller");
  ros::NodeHandle nh("~");

  using namespace giskard_examples;

  WholeBodyController wbc(nh);
  try
  {
    wbc.start(read_params(nh));
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("%s", e.what());
  }

  return 0;
}
