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

#include "RP_move.h"

#include <string>
#include <vector>

/* The implementation of RP_move.h */

/* constructor */
RP_move::RP_move(ros::NodeHandle& nh) : nh_(nh), Action("move_to"), action_client_("/move_base", false)
{
  srv_goal_ = nh_.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  message_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::DoTalk>("/pepper_basic_capabilities/talk", 1);
  engage_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::EngageMode>("/pepper_basic_capabilities/engage_mode");
  web_srv = nh_.serviceClient<pepper_basic_capabilities_msgs::ShowWeb>("/pepper_basic_capabilities/show_tablet_web");
}

void RP_move::talk(std::string s)
{
  pepper_basic_capabilities_msgs::DoTalk srv;
  srv.request.sentence = s;
  message_srv.call(srv);
}

void RP_move::activateCode()
{
  /* HRI tablet */
  //pepper_basic_capabilities_msgs::ShowWeb w_srv;
  //w_srv.request.url = "common/navigating.html";
  //web_srv.call(w_srv);

  while (!action_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("[move_to] Waiting for the move_base action server to come up");
  }

  std::string wpID;
  bool found = false;
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("to"))
    {
      wpID = last_msg_.parameters[i].value;
      found = true;
    }
  }
  std::vector<boost::shared_ptr<geometry_msgs::Pose> > results;
  topological_navigation_msgs::GetLocation srv;
  srv.request.waypoint = wpID;
  if (srv_goal_.call(srv))
  {
    results.push_back(boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose(srv.response.position)));
  }
  else
  {
    ROS_ERROR("Failed to call service /topological_nav/get_location");
    setFail();
    return;
  }

  goal_pose_.pose = *(results[0]);
  // std::string speech_msg;
  // speech_msg = "Nice! I will navigate to the next waypoint.";
  // speech_msg = "Moviéndome!.";
  // talk(speech_msg);
  attentionOn();

  ROS_INFO("[move_to]Commanding to [%s] (%f %f)", wpID.c_str(), goal_pose_.pose.position.x, goal_pose_.pose.position.y);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = goal_pose_;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  action_client_.sendGoal(goal);
}

void RP_move::deActivateCode()
{
  action_client_.cancelAllGoals();
}

void RP_move::attentionOn()
{
  engage_msg_.request.mode = "off";
  engage_srv.call(engage_msg_);
}

void RP_move::step()
{
  bool finished_before_timeout = action_client_.waitForResult(ros::Duration(0.5));
  actionlib::SimpleClientGoalState state = action_client_.getState();
  ROS_INFO("KCL: (%s) action state: %s", params.name.c_str(), state.toString().c_str());
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = action_client_.getState();
    ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      setSuccess();
      return;
    }
    else
    {
      setFail();
      return;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_move_to");
  ros::NodeHandle nh("~");

  std::string actionserver;
  nh.param("action_server", actionserver, std::string("/move_base"));

  RP_move rpmb(nh);

  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}
