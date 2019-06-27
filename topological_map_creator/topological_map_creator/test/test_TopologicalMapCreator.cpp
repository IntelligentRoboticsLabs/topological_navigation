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

/* Author: Jonatan Ginés jgines@gsyc.urjc.es */

/* Mantainer: Jonatan Gines jgines@gsyc.urjc.es */

#include "topological_map_creator/tm_creator_lib.h"
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/package.h>
#include <tf/tf.h>

TEST(TESTSuite, add_wp)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  std_msgs::String msg;
  msg.data = "wp_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();
  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();
  EXPECT_EQ(list.size(), 1);
}

TEST(TESTSuite, add_wps)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  std_msgs::String msg;
  msg.data = "wp_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  msg.data = "wp_2";
  add_wp_pub_.publish(msg);
  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  msg.data = "wp_3";
  add_wp_pub_.publish(msg);
  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();
  EXPECT_EQ(list.size(), 3);
}

TEST(TESTSuite, add_wp_same_name)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  std_msgs::String msg;
  msg.data = "wp_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  add_wp_pub_.publish(msg);
  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();
  EXPECT_EQ(list.size(), 1);
}

TEST(TESTSuite, add_door)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_door_marker", 1);
  std_msgs::String msg;
  msg.data = "door_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();
  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();
  EXPECT_EQ(list.size(), 4);
}

TEST(TESTSuite, rm_wp)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  ros::Publisher rm_wp_pub_ = nh.advertise<std_msgs::Empty>("/tm_creator/rm_marker", 1);

  std_msgs::String add_msg;
  add_msg.data = "wp_1";
  add_wp_pub_.publish(add_msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  tm_creator.clicked_marker_ = "wp_1";
  std_msgs::Empty rm_msg;
  rm_wp_pub_.publish(rm_msg);

  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();
  EXPECT_EQ(list.size(), 0);
}

TEST(TESTSuite, rm_door)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_door_marker", 1);
  ros::Publisher rm_wp_pub_ = nh.advertise<std_msgs::Empty>("/tm_creator/rm_marker", 1);

  std_msgs::String add_msg;
  add_msg.data = "door_1";
  add_wp_pub_.publish(add_msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  tm_creator.clicked_marker_ = "door_1";
  std_msgs::Empty rm_msg;
  rm_wp_pub_.publish(rm_msg);

  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();
  EXPECT_EQ(list.size(), 0);
}

TEST(TESTSuite, link_wps)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  std_msgs::String msg;
  msg.data = "wp_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  msg.data = "wp_2";
  add_wp_pub_.publish(msg);
  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  tm_creator.createLink("wp_1", "wp_2");

  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();

  for (std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker>::iterator it = list.begin(); it != list.end(); it++)
  {
    if (it->marker.name == "wp_1")
    {
      EXPECT_EQ(it->connections.size(), 1);
      for (std::vector<std::string>::iterator it_conn = it->connections.begin();
        it_conn != it->connections.end(); it_conn++)
      {
        EXPECT_EQ(*it_conn, "wp_2");
      }
    }
  }
}

TEST(TESTSuite, link_wp_to_door)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  ros::Publisher add_door_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_door_marker", 1);

  std_msgs::String msg;
  msg.data = "wp_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  msg.data = "door_1";
  add_door_pub_.publish(msg);
  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();


  tm_creator.createLink("door_1_1", "wp_1");

  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();

  for (std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker>::iterator it = list.begin(); it != list.end(); it++)
  {
    if (it->marker.name == "door_1_1")
    {
      EXPECT_EQ(it->connections.size(), 2);
      EXPECT_TRUE(std::find(it->connections.begin(), it->connections.end(), "door_1_2") != it->connections.end());
      EXPECT_TRUE(std::find(it->connections.begin(), it->connections.end(), "wp_1") != it->connections.end());
    }
  }
}

TEST(TESTSuite, link_wp_to_door_2)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  ros::Publisher add_door_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_door_marker", 1);

  std_msgs::String msg;
  msg.data = "wp_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  msg.data = "door_1";
  add_door_pub_.publish(msg);
  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();


  tm_creator.createLink("door_1_1", "wp_1");

  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();

  for (std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker>::iterator it = list.begin(); it != list.end(); it++)
  {
    if (it->marker.name == "door_1_4")
    {
      EXPECT_TRUE(std::find(it->connections.begin(), it->connections.end(), "wp_1") != it->connections.end());
    }
  }
}

TEST(TESTSuite, rm_linked_wp)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  ros::Publisher rm_wp_pub_ = nh.advertise<std_msgs::Empty>("/tm_creator/rm_marker", 1);
  std_msgs::String msg;
  msg.data = "wp_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  msg.data = "wp_2";
  add_wp_pub_.publish(msg);
  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  tm_creator.createLink("wp_1", "wp_2");
  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();

  for (std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker>::iterator it = list.begin(); it != list.end(); it++)
  {
    if (it->marker.name == "wp_1")
    {
      EXPECT_EQ(it->connections.size(), 1);
      for (std::vector<std::string>::iterator it_conn = it->connections.begin();
        it_conn != it->connections.end(); it_conn++)
      {
        EXPECT_EQ(*it_conn, "wp_2");
      }
    }
  }

  tm_creator.clicked_marker_ = "wp_2";
  std_msgs::Empty rm_msg;
  rm_wp_pub_.publish(rm_msg);

  t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();

  list = tm_creator.getMarkerList();

  for (std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker>::iterator it = list.begin(); it != list.end(); it++)
  {
    if (it->marker.name == "wp_1")
    {
      for (std::vector<std::string>::iterator it_conn = it->connections.begin();
        it_conn != it->connections.end(); it_conn++)
      {
        EXPECT_NE(*it_conn, "wp_2");
      }
    }
    EXPECT_NE(it->marker.name, "wp_2");
  }
}

TEST(TESTSuite, update_pos)
{
  tm_creator_lib::TMCreatorLib tm_creator;
  ros::NodeHandle nh;
  ros::Publisher add_wp_pub_ = nh.advertise<std_msgs::String>("/tm_creator/add_wp_marker", 1);
  std_msgs::String msg;
  msg.data = "wp_1";
  add_wp_pub_.publish(msg);
  ros::Time t = ros::Time::now();
  while (t + ros::Duration(0.1) > ros::Time::now()) ros::spinOnce();
  geometry_msgs::Pose p1;
  p1.position.x = 5.0;
  p1.position.y = 5.0;
  p1.position.z = 0.0;
  tm_creator.updatePos("wp_1", p1);

  std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker> list = tm_creator.getMarkerList();

  for (std::vector<tm_creator_lib::TMCreatorLib::ConnectedMarker>::iterator it = list.begin(); it != list.end(); it++)
  {
    if (it->marker.name == "wp_1")
    {
      EXPECT_EQ(it->marker.pose.position.x, 5.0);
      EXPECT_EQ(it->marker.pose.position.y, 5.0);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_TopologicalMapCreator");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
