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

/* Author: Jonatan Gines jgines@gsyc.urjc.es */

/* Mantainer: Jonatan Gines jgines@gsyc.urjc.es */
#ifndef TMCREATORLIB_H
#define TMCREATORLIB_H

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ros/package.h>
#include <topological_map_creator_msgs/AddDoor.h>
#include <topological_map_creator_msgs/AddWp.h>
#include <topological_map_creator_msgs/SaveMap.h>
#include "yaml-cpp/yaml.h"

using topological_map_creator_msgs::AddWp;
using topological_map_creator_msgs::AddDoor;
using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;
using visualization_msgs::Marker;
using std::to_string;

namespace tm_creator_lib
{

const float DOOR_GAP = 0.5;

class TMCreatorLib
{
public:
  TMCreatorLib();
  struct ConnectedMarker
  {
    InteractiveMarker marker;
    std::string room;
    std::vector<std::string> connections;
    std::vector<std::string> connections_to_me;
  };

  struct Way
  {
    std::string id;
    std::string wp_1;
    std::string wp_2;
  };

  struct Door
  {
    Way way_1;
    Way way_2;
  };
  std::string clicked_marker_;
  void step();
  void wpFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void doorFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> getMarkerList();
  void createLink(std::string from, std::string to);
  void updatePos(std::string marker_name, geometry_msgs::Pose p);

private:
  ros::NodeHandle nh_;
  ros::Subscriber add_wp_sub_, rm_sub_, add_door_sub_, create_link_sub_;
  ros::ServiceServer save_srv_, load_srv_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  interactive_markers::MenuHandler menu_handler_;
  std::vector<Door> door_ids_;
  std::vector<ConnectedMarker> marker_list_;
  bool create_link_mode_;

  void init_menu_handler();
  void addWp(std::string wp_name, std::string room, geometry_msgs::Pose p);
  void addDoor(AddDoor door_msg, geometry_msgs::Pose p);
  void addWpCB(const AddWp::ConstPtr& msg);
  void addDoorCB(const AddDoor::ConstPtr& door);
  bool saveSrv(topological_map_creator_msgs::SaveMap::Request &req,
                topological_map_creator_msgs::SaveMap::Response &res);
  bool loadSrv(topological_map_creator_msgs::SaveMap::Request &req,
                topological_map_creator_msgs::SaveMap::Response &res);
  void rmCB(const std_msgs::Empty::ConstPtr& msg);
  void linkCB(const std_msgs::Empty::ConstPtr& msg);
  void rmClickMarker();
  void rmVisualLinkers(std::string name,
    std::vector<std::string> connections,
    std::vector<std::string> connections_to_self);
  std::string getPairId(std::string marker_in);
  Door getDoor(std::string id);
  void updateDes(InteractiveMarker &int_marker);
  void updateLinkersPos(std::string marker_name, geometry_msgs::Point p);

  void connectWps(std::string from, std::string to);
  void insertConnMarker(InteractiveMarker i, std::string room);
  void rmConnMarker(std::string name);
  geometry_msgs::Pose invertPose(geometry_msgs::Pose p_in);
  int findConnMarker(std::string name, ConnectedMarker& c);
  bool findConnMarker(std::string name);
  InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg);
  Marker makeBox(InteractiveMarker &msg);
  Marker makeArrow(InteractiveMarker &msg);
  Marker makeArrowLink(geometry_msgs::Point p1, geometry_msgs::Point p2);
  Marker makeDoorLine(InteractiveMarker &msg);
  void createVisualLink(std::string from, std::string to);
  InteractiveMarker makeWp(
    std::string wp_name,
    std::string room,
    std::string description,
    tf::Vector3 position,
    double yaw);
  void prepareVector(std::vector<std::string> &vector);
  void printConnectionPairs(YAML::Emitter& out, std::string key, std::map<std::string, std::vector<std::string>> map);
  bool saveMap(std::string path);
  void loadMap(std::string path);
  std::vector<std::string> spliter(std::string str, std::string delimiter);
  geometry_msgs::Pose posStrToMsg(std::string str);
};
}  //  namespace tm_creator_lib
#endif  // TMCREATORLIB_H
