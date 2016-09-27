/*
* Copyright (C) 2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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


#ifndef __GISKARD_EXAMPLES_MOTION_PROJECTION_HPP__
#define __GISKARD_EXAMPLES_MOTION_PROJECTION_HPP__

#include <giskard_msgs/WholeBodyProjectionGoal.h>

namespace giskard_examples
{
  class MotionProjector
  {
    public:
      void reset(const giskard_msgs::WholeBodyProjectionGoal& goal);
      bool finished() const;
      void step();
      std::vector<sensor_msgs::JointState> get_results() const;

    private:
      std::vector<sensor_msgs::JointState> results_;
      giskard_msgs::WholeBodyProjectionGoal goal_;
  };
}

#endif // __GISKARD_EXAMPLES_MOTION_PROJECTION__HPP
