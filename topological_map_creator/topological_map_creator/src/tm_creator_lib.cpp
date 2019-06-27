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

#include "topological_map_creator/tm_creator_lib.h"
#include <string>
#include <vector>
#include <algorithm>

namespace tm_creator_lib
{

TMCreatorLib::TMCreatorLib(): nh_()
{
  server.reset(new interactive_markers::InteractiveMarkerServer("tm_creator", "", false));
  add_wp_sub_ = nh_.subscribe("/tm_creator/add_wp_marker", 1, &TMCreatorLib::addWpCB, this);
  add_door_sub_ = nh_.subscribe("/tm_creator/add_door_marker", 1, &TMCreatorLib::addDoorCB, this);
  rm_sub_ = nh_.subscribe("/tm_creator/rm_marker", 1, &TMCreatorLib::rmCB, this);
  create_link_sub_ = nh_.subscribe("/tm_creator/create_link", 1, &TMCreatorLib::linkCB, this);
  save_srv_ = nh_.advertiseService("/tm_creator/save_map", &TMCreatorLib::saveSrv, this);
  load_srv_ = nh_.advertiseService("/tm_creator/load_map", &TMCreatorLib::loadSrv, this);
  clicked_marker_ = "";
  create_link_mode_ = false;
}

void TMCreatorLib::addWpCB(const AddWp::ConstPtr& msg)
{
  geometry_msgs::Pose p;
  tf2::Quaternion quat;
  quat.setEuler(0.0,0.0,0.0);
  p.position.x = 0.0;
  p.position.y = 0.0;
  p.orientation = tf2::toMsg(quat);
  addWp(msg->wp_name, msg->room,p);
}

void TMCreatorLib::addDoorCB(const AddDoor::ConstPtr& door)
{
  geometry_msgs::Pose p;
  tf2::Quaternion quat;
  quat.setEuler(0.0,0.0,0.0);
  p.position.x = 0.0;
  p.position.y = 0.0;
  p.orientation = tf2::toMsg(quat);
  addDoor(*door, p);
}

void TMCreatorLib::rmCB(const std_msgs::Empty::ConstPtr& msg)
{
  rmClickMarker();
}

bool TMCreatorLib::saveSrv(topological_map_creator_msgs::SaveMap::Request &req,
                        topological_map_creator_msgs::SaveMap::Response &res)
{
  return saveMap(req.path);
}

bool TMCreatorLib::loadSrv(topological_map_creator_msgs::SaveMap::Request &req,
                        topological_map_creator_msgs::SaveMap::Response &res)
{
  loadMap(req.path);
  return true;
}

void TMCreatorLib::linkCB(const std_msgs::Empty::ConstPtr& msg)
{
  create_link_mode_ = true;
}

void TMCreatorLib::wpFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  InteractiveMarker int_marker;
  server->get(feedback->marker_name, int_marker);
  updatePos(feedback->marker_name, feedback->pose);
  updateDes(int_marker);
  server->insert(int_marker);
  server->applyChanges();
  if (clicked_marker_ == feedback->marker_name)
    create_link_mode_ = false;

  if (create_link_mode_)
  {
    std::string from = clicked_marker_;
    std::string to = feedback->marker_name;
    Door door_1 = getDoor(from);
    if (from == door_1.way_1.wp_1)
      from = door_1.way_2.wp_2;
    createLink(from, to);
    createVisualLink(from, to);
  }
  else
    updateLinkersPos(feedback->marker_name, feedback->pose.position);

  create_link_mode_ = false;
  clicked_marker_ = feedback->marker_name;
}

void TMCreatorLib::doorFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  InteractiveMarker int_marker_1, int_marker_2, int_marker_3;
  server->get(feedback->marker_name, int_marker_1);
  updatePos(feedback->marker_name, feedback->pose);
  updateDes(int_marker_1);
  Door door_1 = getDoor(feedback->marker_name);
  float angle = tf::getYaw(feedback->pose.orientation);
  if (feedback->marker_name == door_1.way_1.wp_2)
    angle = angle + M_PI;
  float x_wp, y_wp, x_door, y_door;
  x_wp = DOOR_GAP * cos(angle);
  y_wp = DOOR_GAP * sin(angle);
  x_door = DOOR_GAP / 2 * cos(angle);
  y_door = DOOR_GAP / 2 * sin(angle);

  server->get(getPairId(feedback->marker_name), int_marker_2);
  Door door = getDoor(feedback->marker_name);
  server->get(door.way_1.id + "_line", int_marker_3);

    tf::pointTFToMsg(
    tf::Vector3(
      feedback->pose.position.x + x_wp,
      feedback->pose.position.y + y_wp,
      0),
    int_marker_2.pose.position);
  tf::pointTFToMsg(
    tf::Vector3(
      feedback->pose.position.x + x_door,
      feedback->pose.position.y + y_door,
      0),
    int_marker_3.pose.position);
  int_marker_2.pose.orientation = feedback->pose.orientation;
  int_marker_3.pose.orientation = feedback->pose.orientation;

  updatePos(int_marker_2.name, int_marker_2.pose);
  updateDes(int_marker_2);

  geometry_msgs::Pose pose_1 = invertPose(int_marker_1.pose);
  geometry_msgs::Pose pose_2 = invertPose(int_marker_2.pose);
  if (feedback->marker_name == door.way_1.wp_1)
  {
    updatePos(door.way_2.wp_1, pose_2);
    updatePos(door.way_2.wp_2, pose_1);
  }
  else
  {
    updatePos(door.way_2.wp_1, pose_1);
    updatePos(door.way_2.wp_2, pose_2);
  }

  server->insert(int_marker_1);
  server->insert(int_marker_2);
  server->insert(int_marker_3);
  server->applyChanges();
  if (clicked_marker_ == feedback->marker_name)
    create_link_mode_ = false;

  if (create_link_mode_)
  {
    std::string from = clicked_marker_;
    std::string to = feedback->marker_name;
    if (to == door_1.way_1.wp_2)
      to = door_1.way_2.wp_1;
    createLink(from, to);
    createVisualLink(from, to);
  }
  else
  {
    ConnectedMarker c1, c2;
    findConnMarker(door.way_2.wp_1, c1);
    findConnMarker(door.way_2.wp_2, c2);
    updateLinkersPos(feedback->marker_name, feedback->pose.position);
    updateLinkersPos(int_marker_2.name, int_marker_2.pose.position);
    updateLinkersPos(door.way_2.wp_1, c1.marker.pose.position);
    updateLinkersPos(door.way_2.wp_2, c2.marker.pose.position);
  }

  create_link_mode_ = false;
  clicked_marker_ = feedback->marker_name;
}

Marker TMCreatorLib::makeBox(InteractiveMarker &msg)
{
  Marker marker;
  marker.type = Marker::CYLINDER;
  marker.scale.x = msg.scale * 0.2;
  marker.scale.y = msg.scale * 0.2;
  marker.scale.z = 0.01;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  return marker;
}

Marker TMCreatorLib::makeArrowLink(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  Marker marker;
  marker.type = Marker::ARROW;
  marker.color.r = 0.4;
  marker.color.g = 1.0;
  marker.color.b = 0.4;
  marker.color.a = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  return marker;
}

Marker TMCreatorLib::makeArrow(InteractiveMarker &msg)
{
  Marker marker;
  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale * 0.4;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = 0.01;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  return marker;
}

Marker TMCreatorLib::makeDoorLine(InteractiveMarker &msg)
{
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = 0.15;
  marker.scale.y = 0.8;
  marker.scale.z = 0.01;
  marker.color.r = 0.56;
  marker.color.g = 0.43;
  marker.color.b = 0.24;
  marker.color.a = 1.0;
  return marker;
}

bool TMCreatorLib::findConnMarker(std::string name)
{
  for (std::vector<ConnectedMarker>::iterator it = marker_list_.begin(); it != marker_list_.end(); it++)
  {
    if (it->marker.name == name)
      return true;
  }
  return false;
}

int TMCreatorLib::findConnMarker(std::string name, ConnectedMarker& c)
{
  for (std::vector<ConnectedMarker>::iterator it = marker_list_.begin(); it != marker_list_.end(); it++)
  {
    if (it->marker.name == name)
    {
      c = *it;
      return it - marker_list_.begin();
    }
  }
  return -1;
}

std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> TMCreatorLib::getMarkerList()
{
  return marker_list_;
}

void TMCreatorLib::insertConnMarker(InteractiveMarker i, std::string room)
{
  ConnectedMarker c_marker_1;
  c_marker_1.marker = i;
  c_marker_1.room = room;
  marker_list_.push_back(c_marker_1);
}

void TMCreatorLib::rmConnMarker(std::string name)
{
  std::vector<std::string>::iterator it;
  std::vector<std::string> connections, connections_to_me;
  for (int i = 0; i < marker_list_.size(); i++)
  {
    if (marker_list_[i].marker.name == name)
    {
      connections = marker_list_[i].connections;
      connections_to_me = marker_list_[i].connections_to_me;
      marker_list_.erase(marker_list_.begin() + i);
    }
    else if (std::find(marker_list_[i].connections.begin(),
      marker_list_[i].connections.end(), name) != marker_list_[i].connections.end())
    {
      it = std::find(marker_list_[i].connections.begin(), marker_list_[i].connections.end(), name);
      marker_list_[i].connections.erase(it);
    }
  }
  rmVisualLinkers(name, connections, connections_to_me);
}

void TMCreatorLib::connectWps(std::string from, std::string to)
{
  for (std::vector<ConnectedMarker>::iterator it = marker_list_.begin(); it != marker_list_.end(); it++)
  {
    if (it->marker.name == from)
    {
      ConnectedMarker& c = *it;
      c.connections.push_back(to);
    }
    else if (it->marker.name == to)
    {
      ConnectedMarker& c = *it;
      c.connections_to_me.push_back(from);
    }
  }
}

geometry_msgs::Pose TMCreatorLib::invertPose(geometry_msgs::Pose p_in)
{
  geometry_msgs::Pose p_out = p_in;
  tf::quaternionTFToMsg(
    tf::createQuaternionFromYaw(tf::getYaw(p_in.orientation) - M_PI).normalize(),
    p_out.orientation);
  return p_out;
}

void TMCreatorLib::createLink(std::string from, std::string to)
{
  connectWps(from, to);
}

void TMCreatorLib::createVisualLink(std::string from, std::string to)
{
  ConnectedMarker c1, c2;
  findConnMarker(from, c1);
  findConnMarker(to, c2);
  geometry_msgs::Point p1, p2;
  p1.x = c1.marker.pose.position.x;
  p1.y = c1.marker.pose.position.y;
  p2.x = c2.marker.pose.position.x;
  p2.y = c2.marker.pose.position.y;
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  int_marker.name = "link_" + from + "_to_" + to;
  InteractiveMarkerControl control;
  control.markers.push_back(makeArrowLink(p1, p2));
  control.always_visible = true;
  int_marker.controls.push_back(control);
  server->insert(int_marker);
  server->applyChanges();
}

std::string TMCreatorLib::getPairId(std::string marker_in)
{
  for (std::vector<Door>::iterator it = door_ids_.begin(); it != door_ids_.end(); it++)
  {
    if (it->way_1.wp_1 == marker_in || it->way_1.wp_2 == marker_in)
    {
      std::string s = it->way_1.id;
      marker_in.erase(0, s.length());
      if (marker_in == "_1")
        return s + "_2";
      else
        return s + "_1";
    }
  }
  ROS_ERROR("[getPairId] Pair id not found");
  return "";
}


tm_creator_lib::TMCreatorLib::Door TMCreatorLib::getDoor(std::string id)
{
  Door d;
  for (std::vector<Door>::iterator it = door_ids_.begin(); it != door_ids_.end(); it++)
  {
    if (it->way_1.id == id || it->way_2.id == id || it->way_1.wp_1 == id ||
      it->way_1.wp_2 == id ||it->way_2.wp_1 == id ||it->way_2.wp_2 == id)
      return *it;
  }
  return d;
}

void TMCreatorLib::updateDes(InteractiveMarker &int_marker)
{
  ConnectedMarker c_marker;
  findConnMarker(int_marker.name, c_marker);
  if (int_marker.description.find(int_marker.name) != std::string::npos)
    int_marker.description = int_marker.name +
    "\n" + c_marker.room +
    "\n( "
    + std::to_string(c_marker.marker.pose.position.x)
    + " ,"
    + std::to_string(c_marker.marker.pose.position.y)
    + " ,0.0 )";
}

void TMCreatorLib::updatePos(std::string marker_name, geometry_msgs::Pose p)
{
  ConnectedMarker c;
  int index = findConnMarker(marker_name, c);
  c.marker.pose = p;
  marker_list_.at(index) = c;
}

void TMCreatorLib::updateLinkersPos(std::string marker_name, geometry_msgs::Point p)
{
  std::vector<std::string> connections;
  std::vector<std::string> connections_to_me;
  InteractiveMarker int_marker;
  ConnectedMarker c_marker;
  findConnMarker(marker_name, c_marker);
  connections = c_marker.connections;
  connections_to_me = c_marker.connections_to_me;

  for (std::vector<ConnectedMarker>::iterator it = marker_list_.begin(); it != marker_list_.end(); it++)
  {
    if (std::find(connections.begin(), connections.end(), it->marker.name) != connections.end())
    {
      if (!server->get("link_" + marker_name + "_to_" + it->marker.name, int_marker))
        continue;
      InteractiveMarkerControl control;
      control.markers.push_back(makeArrowLink(p, it->marker.pose.position));
      control.always_visible = true;
      std::vector<InteractiveMarkerControl> control_vector;
      control_vector.push_back(control);
      int_marker.controls = control_vector;
      server->insert(int_marker);
    }

    if (std::find(connections_to_me.begin(), connections_to_me.end(), it->marker.name) != connections_to_me.end())
    {
      if (!server->get("link_" + it->marker.name + "_to_" + marker_name, int_marker))
        continue;
      InteractiveMarkerControl control;
      control.markers.push_back(makeArrowLink(it->marker.pose.position, p));
      control.always_visible = true;
      std::vector<InteractiveMarkerControl> control_vector;
      control_vector.push_back(control);
      int_marker.controls = control_vector;
      server->insert(int_marker);
    }
  }
  server->applyChanges();
}

void TMCreatorLib::init_menu_handler()
{
  menu_handler_.insert("Set link", boost::bind(&TMCreatorLib::wpFeedback, this, _1));
  menu_handler_.insert("Second Entry", boost::bind(&TMCreatorLib::wpFeedback, this, _1));
  /*interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert( "Submenu" );
  menu_handler_.insert( sub_menu_handle, "First Entry", boost::bind(&TMCreatorLib::wpFeedback, this, _1) );
  menu_handler_.insert( sub_menu_handle, "Second Entry", boost::bind(&TMCreatorLib::wpFeedback, this, _1) );*/
}

void TMCreatorLib::rmClickMarker()
{
  bool found = false;
  Door door;
  for (std::vector<Door>::iterator it = door_ids_.begin(); it != door_ids_.end(); it++)
  {
    if (clicked_marker_ == it->way_1.wp_1 || clicked_marker_ == it->way_1.wp_2)
    {
      door = *it;
      found = true;
    }
  }
  if (found)
  {
    server->erase(door.way_1.wp_1);
    server->erase(door.way_1.wp_2);
    server->erase(door.way_1.id + "_line");
    server->applyChanges();

    rmConnMarker(door.way_1.wp_1);
    rmConnMarker(door.way_1.wp_2);
    rmConnMarker(door.way_2.wp_1);
    rmConnMarker(door.way_2.wp_2);
  }
  else
  {
    server->erase(clicked_marker_);
    server->applyChanges();
    rmConnMarker(clicked_marker_);
  }
  clicked_marker_ = "";
}

void TMCreatorLib::rmVisualLinkers(std::string name,
  std::vector<std::string> connections,
  std::vector<std::string> connections_to_me)
{
  for (std::vector<std::string>::iterator it = connections.begin(); it != connections.end(); it++)
    server->erase("link_" + name + "_to_" + *it);

  for (std::vector<std::string>::iterator it = connections_to_me.begin(); it != connections_to_me.end(); it++)
    server->erase("link_" + *it + "_to_" + name);

  server->applyChanges();
}

void TMCreatorLib::addWp(std::string wp_name, std::string room, geometry_msgs::Pose p)
{
  if (findConnMarker(wp_name))
  {
    ROS_ERROR("Marker is already exists with the same name");
    return;
  }

  tf2::Quaternion quat;
  tf2::fromMsg(p.orientation, quat);

  InteractiveMarker int_marker;
  int_marker = makeWp(
    wp_name,
    room,
    "",
    tf::Vector3(p.position.x, p.position.y, 0), quat.getAngle());

  InteractiveMarkerControl control;
  control = int_marker.controls[0];
  control.always_visible = true;
  control.markers.push_back(makeArrow(int_marker));
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->applyChanges();

  server->setCallback(int_marker.name, boost::bind(&TMCreatorLib::wpFeedback, this, _1));
  insertConnMarker(int_marker, room);
}

InteractiveMarker TMCreatorLib::makeWp(
  std::string wp_name,
  std::string room,
  std::string description,
  tf::Vector3 position,
  double yaw)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw).normalize(), int_marker.pose.orientation);
  int_marker.name = wp_name;
  int_marker.scale = 1;
  if (description != "")
    int_marker.description = description;
  else
    int_marker.description = wp_name + "\n" + room +"\n ( 0.0, 0.0, 0.0 )";

  InteractiveMarkerControl control;
  tf::Quaternion orientation(0.0, 1.0, 0.0, 1.0);
  tf::quaternionTFToMsg(orientation.normalize(), control.orientation);
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);
  return int_marker;
}

void TMCreatorLib::addDoor(AddDoor door_msg, geometry_msgs::Pose p)
{
  InteractiveMarker line_marker, int_marker_1, int_marker_2, int_marker_3, int_marker_4;

  Door door;
  door.way_1.id = door_msg.room_1 + "_to_" + door_msg.room_2;
  door.way_2.id = door_msg.room_2 + "_to_" + door_msg.room_1;
  door.way_1.wp_1 = door.way_1.id + "_1";
  door.way_1.wp_2 = door.way_1.id + "_2";
  door.way_2.wp_1 = door.way_2.id + "_1";
  door.way_2.wp_2 = door.way_2.id + "_2";

  tf2::Quaternion quat;
  tf2::fromMsg(p.orientation, quat);

  int_marker_1 = makeWp(
    door.way_1.wp_1,
    door_msg.room_1,
    "room == " + door_msg.room_1,
    tf::Vector3(p.position.x, p.position.y, 0),quat.getAngle());
  int_marker_2 = makeWp(
    door.way_1.wp_2, door_msg.room_2,
    "room == " + door_msg.room_2,
    tf::Vector3(p.position.x + DOOR_GAP, p.position.y, 0),quat.getAngle());
  int_marker_3 = makeWp(
    door.way_2.wp_1,
    door_msg.room_2,
    "",
    tf::Vector3(p.position.x + DOOR_GAP, p.position.y, 0), quat.getAngle() + M_PI);
  int_marker_4 = makeWp(
    door.way_2.wp_2,
    door_msg.room_1,
    "",
    tf::Vector3(p.position.x, p.position.y, 0), quat.getAngle() + M_PI);
  server->insert(int_marker_1);
  server->insert(int_marker_2);

  line_marker.header.frame_id = "map";
  tf::Vector3 position;
  position = tf::Vector3(p.position.x + DOOR_GAP/2, p.position.y, 0);
  tf::pointTFToMsg(position, line_marker.pose.position);
  line_marker.name = door.way_1.id + "_line";
  line_marker.scale = 1;

  InteractiveMarkerControl control;
  tf::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);

  Marker line = makeDoorLine(line_marker);
  control.markers.push_back(line);
  control.always_visible = true;
  line_marker.controls.push_back(control);
  server->insert(line_marker);

  server->applyChanges();
  server->setCallback(door.way_1.wp_1, boost::bind(&TMCreatorLib::doorFeedback, this, _1));
  server->setCallback(door.way_1.wp_2, boost::bind(&TMCreatorLib::doorFeedback, this, _1));
  door_ids_.push_back(door);

  insertConnMarker(int_marker_1, door_msg.room_1);
  insertConnMarker(int_marker_2, door_msg.room_2);
  insertConnMarker(int_marker_3, door_msg.room_2);
  insertConnMarker(int_marker_4, door_msg.room_1);
}

YAML::Emitter& operator << (YAML::Emitter& out, const tm_creator_lib::TMCreatorLib::ConnectedMarker& c)
{
  out << YAML::BeginMap;
  out << YAML::Key << c.marker.name;
  out << YAML::Value << YAML::Flow <<YAML::BeginMap;
  out << YAML::Key << "room";
  out << YAML::Value << c.room;
  out << YAML::Key << "position";
  out << YAML::Value <<
    to_string(c.marker.pose.position.x) + ", " +
    to_string(c.marker.pose.position.y) + ", " +
    to_string(tf::getYaw(c.marker.pose.orientation));
  out << YAML::EndMap;
  out << YAML::EndMap;
  return out;
}

void TMCreatorLib::prepareVector(std::vector<std::string> &vector)
{
  sort(vector.begin(), vector.end());
  vector.erase(unique(vector.begin(), vector.end()), vector.end());
}

void TMCreatorLib::printConnectionPairs(
  YAML::Emitter& out,
  std::string key,
  std::map<std::string,std::vector<std::string>> map)
{
  out << YAML::BeginMap;
  out << YAML::Key << key;
  out << YAML::Value;
  out << YAML::BeginMap;
  for (std::map<std::string,std::vector<std::string>>::iterator it=map.begin();
    it!=map.end();
    ++it)
  {
    out << YAML::Key << it->first;
    out << YAML::Value << YAML::Flow << it->second;
  }
  out << YAML::EndMap;
  out << YAML::EndMap;
}

bool TMCreatorLib::saveMap(std::string path)
{

  YAML::Emitter out, out_wp, out_conn, out_doors;
  std::ofstream file(path + "/.topological_map.yaml");
  std::vector<std::string> wp_ids, door_ids, room_ids, connections_ids;
  for (std::vector<ConnectedMarker>::iterator it = marker_list_.begin(); it != marker_list_.end(); it++)
  {
    wp_ids.push_back(it->marker.name);
    room_ids.push_back(it->room);
    out_wp << *it;
  }
  prepareVector(room_ids);
  for (std::vector<ConnectedMarker>::iterator it = marker_list_.begin(); it != marker_list_.end(); it++)
  {
    for (std::vector<std::string>::iterator it_conn = it->connections.begin();
      it_conn != it->connections.end(); it_conn++)
    {
      ConnectedMarker c_marker;
      findConnMarker(*it_conn, c_marker);
      if (getDoor(c_marker.marker.name).way_1.id == "" && it->room != c_marker.room)
      {
        std::vector<std::string> rooms, links;
        connections_ids.push_back(it->room + "_to_" + c_marker.room);
        rooms.push_back(it->room);
        rooms.push_back(c_marker.room);
        links.push_back(it->marker.name);
        links.push_back(*it_conn);
        std::map<std::string,std::vector<std::string>> conn_map;
        conn_map["rooms"] = rooms;
        conn_map["visual_link"] = links;
        printConnectionPairs(out_conn, it->room + "_to_" + c_marker.room, conn_map);
      }
    }
  }
  prepareVector(connections_ids);
  for (std::vector<Door>::iterator it = door_ids_.begin(); it != door_ids_.end(); it++)
  {
    door_ids.push_back(it->way_1.id);
    door_ids.push_back(it->way_2.id);

    std::vector<std::string> waypoints_1, waypoints_2;
    std::map<std::string,std::vector<std::string>> wp_map_1, wp_map_2;

    waypoints_1.push_back(it->way_1.id + "_1");
    waypoints_1.push_back(it->way_1.id + "_2");

    wp_map_1["waypoints"] = waypoints_1;
    printConnectionPairs(out_doors, it->way_1.id, wp_map_1);

    waypoints_2.push_back(it->way_2.id + "_1");
    waypoints_2.push_back(it->way_2.id + "_2");
    wp_map_2["waypoints"] = waypoints_2;
    printConnectionPairs(out_doors, it->way_2.id, wp_map_2);
  }

  out << YAML::BeginMap;
  out << YAML::Key << "room_ids";
  out << YAML::Value << YAML::Flow << room_ids;
  out << YAML::Key << "door_ids";
  out << YAML::Value << YAML::Flow << door_ids;
  out << YAML::Key << "connections";
  out << YAML::Value << YAML::Flow << connections_ids;
  out << YAML::Key << "waypoints_ids";
  out << YAML::Value << YAML::Flow << wp_ids;
  out << YAML::EndMap;

  file << out.c_str();
  file << '\n';
  file << out_conn.c_str();
  file << '\n';
  file << out_doors.c_str();
  file << '\n';
  file << out_wp.c_str();
  file.close();

  std::ifstream file_in(path + "/.topological_map.yaml");
  std::ofstream file_out(path + "/topological_map.yaml");

  std::string str;
  while(getline(file_in,str))
  {
    if (str != "---")
    {
      file_out << str;
      file_out << '\n';
    }
  }
  file_in.close();
  remove((path + "/.topological_map.yaml").c_str());
  file_out.close();
  return true;
}

/*void operator >> (const YAML::Node& node, ValuesTable::TemporalInfo& info) {
  for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
  {
    YAML::Node values;
    values = it->second;
    for (int i=0; i< values.size();i++)
      info[it->first.as<int>()].push_back(values[i].as<float>());
  }
}
*/

std::vector<std::string> TMCreatorLib::spliter(std::string str, std::string delimiter)
{
  std::vector<std::string> v;
  size_t token = 0;

  while (token != std::string::npos)
  {
    token = str.find(delimiter, 0);
    v.push_back(str.substr(0, token));
    str.erase(0, str.find(delimiter) + delimiter.length());
  }
  return v;
}

geometry_msgs::Pose TMCreatorLib::posStrToMsg(std::string str)
{
  geometry_msgs::Pose p;
  std::vector<std::string> v = spliter(str, ",");
  p.position.x = std::atof(v[0].c_str());
  p.position.y = std::atof(v[1].c_str());
  tf2::Quaternion quat;
  quat.setEuler(std::atof(v[2].c_str()),0.0,0.0);
  p.orientation = tf2::toMsg(quat);
  return p;
}

void TMCreatorLib::loadMap(std::string path)
{
  std::string from, wp1, wp2, room, position, room_door_1, room_door_2;
  geometry_msgs::Pose p;
  size_t first_token = 0;
  std::ifstream fin(path);
  YAML::Parser parser(fin);
  YAML::Node root;
  std::vector<std::string> wp_ids, door_ids, rooms, connections;
   try {
       root = YAML::LoadFile(path);
   } catch (const std::exception& e){
       std::cout << e.what() << "\n";
       return;
   }
   for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
   {
     if(it->first.as<std::string>() == "waypoints_ids")
     {
       for (int i = 0; i < it->second.size(); i++)
         wp_ids.push_back(it->second[i].as<std::string>());
     }
     else if (it->first.as<std::string>() == "door_ids")
     {
       for (int i = 0; i < it->second.size(); i++)
         door_ids.push_back(it->second[i].as<std::string>());
     }
     else if (it->first.as<std::string>() == "connections")
     {
       for (int i = 0; i < it->second.size(); i++)
         connections.push_back(it->second[i].as<std::string>());
     }
   }

   for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
   {
     if(std::find(wp_ids.begin(), wp_ids.end(), it->first.as<std::string>()) != wp_ids.end())
     {
       from = it->first.as<std::string>();
       if (std::find(door_ids.begin(), door_ids.end(), from.substr(0, from.length()-2)) == door_ids.end())
       {
         room = it->second["room"].as<std::string>();
         addWp(from,room,posStrToMsg(it->second["position"].as<std::string>()));
       }
       else
       {
         //ROS_INFO("%s %s", from.c_str(), it->second["position"].as<std::string>().c_str());
         from = from.substr(0, from.length()-2);
         std::vector<std::string> r = spliter(from, "_to_");
         //ROS_INFO("%s %s", r[0].c_str(), r[1].c_str());
         if (rooms.size() == 0)
         {
           rooms.insert(rooms.end(),r.begin(),r.end());
           AddDoor msg;
           msg.room_1 = r[0];
           msg.room_2 = r[1];
           addDoor(msg, posStrToMsg(it->second["position"].as<std::string>()));
         }
         else if (std::find(rooms.begin(), rooms.end(), r[0]) == rooms.end())
         {
           rooms.insert(rooms.end(),r.begin(),r.end());
           AddDoor msg;
           msg.room_1 = r[0];
           msg.room_2 = r[1];
           addDoor(msg, posStrToMsg(it->second["position"].as<std::string>()));
         }
       }
     }else if(std::find(
       connections.begin(),
       connections.end(),
       it->first.as<std::string>()) != connections.end())
     {
       YAML::Node link_node = root[it->first.as<std::string>()];
       for (YAML::const_iterator it_links=link_node.begin();it_links != link_node.end();++it_links)
         if (it_links->first.as<std::string>() == "visual_link")
         {
           std::vector<std::string> wps = it_links->second.as<std::vector<std::string>>();
           createLink(wps[0], wps[1]);
           createVisualLink(wps[0], wps[1]);
         }
     }
   }
}

}  //  namespace tm_creator_lib
