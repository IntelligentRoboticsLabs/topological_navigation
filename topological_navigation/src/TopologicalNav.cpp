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

#include "topological_navigation/TopologicalNav.h"

#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>

namespace topological_navigation
{
TopologicalNav::TopologicalNav() : nh_("~")
{
  start_location();

  service_locs_ = nh_.advertiseService("get_location", &TopologicalNav::get_location, this);
  service_setlocs_ = nh_.advertiseService("set_location", &TopologicalNav::set_location, this);
}

bool TopologicalNav::get_location(topological_navigation_msgs::GetLocation::Request& req,
                                  topological_navigation_msgs::GetLocation::Response& res)
{
  geometry_msgs::Pose p;
  if (get_location(req.waypoint, p))
  {
    res.position = p;
    return true;
  }
  else
    return false;
}

bool TopologicalNav::set_location(topological_navigation_msgs::SetLocation::Request& req,
                                  topological_navigation_msgs::SetLocation::Response& res)
{
  return set_location(req.waypoint, req.position);
}

bool TopologicalNav::get_location(std::string location, geometry_msgs::Pose& p)
{
  if (waypoints_pos_.find(location) != waypoints_pos_.end())
  {
    p = waypoints_pos_[location];
    return true;
  }
  else
  {
    ROS_ERROR("[%s] not found in topological map", location.c_str());
    return false;
  }
}

bool TopologicalNav::set_location(std::string location, geometry_msgs::Pose p)
{
  waypoints_pos_[location] = p;
  return true;
}

geometry_msgs::Pose TopologicalNav::stringToPose(const std::string& coords)
{
  double lx, ly, lz;
  sscanf(coords.c_str(), "%lf, %lf, %lf", &lx, &ly, &lz);

  geometry_msgs::Pose pose;
  pose.position.x = lx;
  pose.position.y = ly;
  pose.position.z = 0;

  pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, lz));

  return pose;
}

void TopologicalNav::start_location()
{
  std::vector<std::string> rooms;
  std::vector<std::string> doors;
  std::vector<std::string> waypoints;
  std::vector<std::string> connections;

  if (nh_.hasParam(ros::this_node::getName() + "/room_ids"))
    nh_.getParam(ros::this_node::getName() + "/room_ids", rooms);
  if (nh_.hasParam(ros::this_node::getName() + "/door_ids"))
    nh_.getParam(ros::this_node::getName() + "/door_ids", doors);
  if (nh_.hasParam(ros::this_node::getName() + "/waypoints_ids"))
    nh_.getParam(ros::this_node::getName() + "/waypoints_ids", waypoints);
  if (nh_.hasParam(ros::this_node::getName() + "/connections"))
    nh_.getParam(ros::this_node::getName() + "/connections", connections);

  graph_.begin_batch();
  for (int i = 0; i < rooms.size(); i++)
  {
    add_instance("room", rooms[i]);
    graph_.add_node(rooms[i], "room");
  }

  for (int i = 0; i < doors.size(); i++)
    add_instance("door", doors[i]);

  for (int i = 0; i < waypoints.size(); i++)
  {
    add_instance("waypoint", waypoints[i]);
    graph_.add_node(waypoints[i], "waypoint");
  }

  for (int i = 0; i < waypoints.size(); i++)
  {
    if (nh_.hasParam(ros::this_node::getName() + "/" + waypoints[i]))
    {
      std::string room, coords;
      nh_.getParam(ros::this_node::getName() + "/" + waypoints[i] + "/room", room);
      nh_.getParam(ros::this_node::getName() + "/" + waypoints[i] + "/position", coords);

      waypoints_rooms_[waypoints[i]] = room;

      waypoints_pos_[waypoints[i]] = stringToPose(coords);

      add_predicate("waypoint_at " + waypoints[i] + " " + room);
      graph_.add_edge(room, "waypoint_at", waypoints[i]);

      tf2::Transform room2wp;
      tf2::fromMsg(waypoints_pos_[waypoints[i]], room2wp);

      graph_.add_edge(room, room2wp, waypoints[i], true);
    }
  }

  for (int i = 0; i < doors.size(); i++)
  {
    if (nh_.hasParam(ros::this_node::getName() + "/" + doors[i]))
    {
      std::vector<std::string> wps;
      nh_.getParam(ros::this_node::getName() + "/" + doors[i] + "/waypoints", wps);

      add_predicate("door_opened " + doors[i]);
      add_predicate("door_connected " + doors[i] + " " + waypoints_rooms_[wps[0]] + " " + waypoints_rooms_[wps[1]] +
                    " " + wps[0] + " " + wps[1]);
    }
  }

  for (int i = 0; i < connections.size(); i++)
  {
    if (nh_.hasParam(ros::this_node::getName() + "/" + connections[i]))
    {
      std::vector<std::string> rooms;
      nh_.getParam(ros::this_node::getName() + "/" + connections[i] + "/rooms", rooms);
      add_predicate("free_connected " + rooms[0] + " " + rooms[1]);
      add_predicate("free_connected " + rooms[1] + " " + rooms[0]);
    }
  }

  graph_.flush();
}

}  // namespace topological_navigation
