/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#pragma once
#include <string>

namespace kdl_kinematics_plugin
{
/** \brief A model of a mimic joint. Mimic joints are typically unactuated joints
that are constrained to follow the motion of another joint. The constraint is linear, i.e.
joint_angle_constrained_joint = joint_angle_mimicked_joint*multiplier + offset
*/
class JointMimic
{
public:
  JointMimic()
  {
    reset(0, 0);
  }

  /** \brief Offset for this joint value from the joint that it mimics */
  double offset;
  /** \brief Multiplier for this joint value from the joint that it mimics */
  double multiplier;
  /** \brief For mimic joints: the index of the active joint that this joint mimics in the active 
  joints vector. For active joints: the index of the joint in the active joints vector. */
  unsigned int map_index;
   /** \brief For mimic joints: the index of the active joint that this joint mimics in the (active + mimic)
  joints vector. For active joints: the index of the joint in the (active + mimic) joints vector. */
  unsigned int index;
  /** \brief Name of this joint */
  std::string joint_name;
  /** \brief If true, this joint is an active DOF and not a mimic joint*/
  bool active;

  void reset(unsigned int map_idx, unsigned int idx)
  {
    offset = 0.0;
    multiplier = 1.0;
    map_index = map_idx;
    index = idx;
    active = false;
  }
};
}  // namespace kdl_kinematics_plugin
