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

#include <giskard_examples/motion_projector.hpp>

namespace giskard_examples
{
  void MotionProjector::reset(const giskard_msgs::WholeBodyProjectionGoal& goal)
  {
    goal_ = goal;
    results_.empty();
  }

  bool MotionProjector::finished() const
  {
    return results_.size() == goal_.iterations;
  }

  void MotionProjector::step()
  {
    // TODO: implement me
  }

  std::vector<sensor_msgs::JointState> MotionProjector::get_results() const
  {
    return results_;
  }
}
