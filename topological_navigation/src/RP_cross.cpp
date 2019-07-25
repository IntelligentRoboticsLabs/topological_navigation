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

#include "RP_cross.h"

#include <string>
#include <vector>

/* constructor */
RP_cross::RP_cross(ros::NodeHandle& nh) : nh_(nh), Action("cross"), action_client_("/move_base", false)
{
  srv_goal_ = nh_.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  clear_cmap_srv = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  message_pub = nh_.advertise<std_msgs::String>("/speech", 1);
  sonar_sub = nh_.subscribe("/pepper_robot/sonar/front", 1, &RP_cross::sonarCallback, this);
}

void RP_cross::activateCode()
{
  while (!action_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("[cross] Waiting for the move_base action server to come up");
  }

  goal_sended = false;
  door_msg_sended = false;
  sonar_activate = true;
  state = UNKNOWN;
  timer = nh_.createTimer(ros::Duration(10), &RP_cross::timerCallback, this);

  std::string wpID;
  bool found = false;
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("wp2"))
    {
      wpID = last_msg_.parameters[i].value;
      found = true;
    }
  }
  std::vector<boost::shared_ptr<geometry_msgs::Pose>> results;
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

  ROS_INFO("[cross]Commanding to [%s] (%f %f)", wpID.c_str(), goal_pose_.pose.position.x, goal_pose_.pose.position.y);
  goal.target_pose = goal_pose_;
  goal.target_pose.header.frame_id = "map";  // Quizás esto debería ir en el map y lerobocupo del yaml
}

void RP_cross::deActivateCode()
{
  action_client_.cancelAllGoals();
}

void RP_cross::step()
{
  std_msgs::String speech_msg;
  switch (state)
  {
    case DOOR_OPENED:
      if (!goal_sended)
      {
        speech_msg.data = "Crossing the door";
        // speech_msg.data = "Cruzando la puerta";
        message_pub.publish(speech_msg);
        goal.target_pose.header.stamp = ros::Time::now();
        std_srvs::Empty srv;
        if (!clear_cmap_srv.call(srv))
        {
          ROS_ERROR("Failed to call service /move_base/clear_costmaps");
          return;
        }
        action_client_.sendGoal(goal);
        goal_sended = true;
      }
      else
      {
        bool finished_before_timeout = action_client_.waitForResult(ros::Duration(0.5));
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
            setFail();
          return;
        }
      }
      break;
    case DOOR_CLOSED:
      if (!door_msg_sended)
      {
        speech_msg.data = "Hi, Could you open the door, please?";
        // speech_msg.data = "Hola, ¿Podrias abrirme la puerta, por favor?";
        message_pub.publish(speech_msg);
        door_msg_sended = true;
      }
      break;
    case UNKNOWN:
      break;
  }
}

void RP_cross::timerCallback(const ros::TimerEvent&)
{
  if (!isActive())
    return;

  door_msg_sended = false;
}

void RP_cross::sonarCallback(const sensor_msgs::Range::ConstPtr& sonar_in)
{
  if (!isActive())
    return;

  // ROS_INFO("[sonarCallback] Range -- %f",sonar_in->range);
  if (sonar_activate && (UNKNOWN || DOOR_CLOSED) && sonar_in->range > 1.0 && sonar_in->range > sonar_in->min_range)
  {
    sonar_activate = false;
    state = DOOR_OPENED;
  }
  else if (sonar_activate && UNKNOWN && sonar_in->range < 1.0 && sonar_in->range > sonar_in->min_range)
  {
    state = DOOR_CLOSED;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_cross");
  ros::NodeHandle nh("~");

  std::string actionserver;
  nh.param("action_server", actionserver, std::string("/move_base"));

  RP_cross rpmb(nh);

  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}
