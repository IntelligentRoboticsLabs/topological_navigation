
/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef TOPOLOGICAL_NAVIGATION_TOPOLOGICALNAV_H_
#define TOPOLOGICAL_NAVIGATION_TOPOLOGICALNAV_H_

#include <ros/ros.h>
#include <ir_planning/KMSClient.h>
#include <geometry_msgs/Pose.h>
#include <topological_navigation_msgs/GetLocation.h>
#include <topological_navigation_msgs/SetLocation.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>

#include <map>
#include <string>

namespace topological_navigation
{
class TopologicalNav : public ir_planning::KMSClient
{
public:
  TopologicalNav();

  bool get_location(std::string location, geometry_msgs::Pose& p);
  bool set_location(std::string location, geometry_msgs::Pose p);

  geometry_msgs::Pose stringToPose(const std::string& coords);

private:
  bool get_location(topological_navigation_msgs::GetLocation::Request& req,
                    topological_navigation_msgs::GetLocation::Response& res);
  bool set_location(topological_navigation_msgs::SetLocation::Request& req,
                    topological_navigation_msgs::SetLocation::Response& res);

  void start_location();

  ros::NodeHandle nh_;

  std::map<std::string, std::string> waypoints_rooms_;
  std::map<std::string, geometry_msgs::Pose> waypoints_pos_;

  ros::ServiceServer service_locs_, service_setlocs_, service_move_, service_guide_, service_move_wp_, service_path_;
};

}  // namespace topological_navigation

#endif  // TOPOLOGICAL_NAVIGATION_TOPOLOGICALNAV_H_
