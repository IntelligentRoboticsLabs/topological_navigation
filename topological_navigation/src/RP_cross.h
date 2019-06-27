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

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include "sensor_msgs/Range.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"

#include <bica_planning/Action.h>
#include <topological_navigation_msgs/GetLocation.h>
#include <pepper_basic_capabilities_msgs/EngageMode.h>
#include <pepper_basic_capabilities_msgs/ShowWeb.h>
#include <pepper_basic_capabilities_msgs/DoTalk.h>

#ifndef KCL_cross
#define KCL_cross

class RP_cross : public bica_planning::Action
{
public:
  explicit RP_cross(ros::NodeHandle& nh);

protected:
  void activateCode();
  void deActivateCode();
  void step();

private:
  ros::NodeHandle nh_;

  void timerCallback(const ros::TimerEvent&);
  void sonarCallback(const sensor_msgs::Range::ConstPtr& sonar_in);
  void attentionOn();
  void talk(std::string s);

  ros::Timer timer;
  enum StateType
  {
    DOOR_OPENED,
    DOOR_CLOSED,
    UNKNOWN
  };
  StateType state;
  std::string actionserver_;
  geometry_msgs::PoseStamped goal_pose_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
  ros::ServiceClient srv_goal_, clear_cmap_srv, engage_srv, web_srv, message_srv;
  ros::Publisher message_pub;
  ros::Subscriber sonar_sub;
  move_base_msgs::MoveBaseGoal goal;
  bool goal_sended, door_msg_sended, sonar_activate;
  pepper_basic_capabilities_msgs::EngageMode engage_msg_;
};

#endif
