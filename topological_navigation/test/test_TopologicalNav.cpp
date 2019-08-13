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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/package.h>

#include <tf/tf.h>

TEST(TESTSuite, test_api_locations)
{
  topological_navigation::TopologicalNav map;

  geometry_msgs::Pose aux1;
  EXPECT_FALSE(map.get_location("wp1", aux1));

  geometry_msgs::Pose aux2;
  aux2.orientation = tf2::createQuaternionMsgFromYaw(2.0);
  aux2.position.x = 1.0;

  geometry_msgs::Pose aux3;
  EXPECT_TRUE(map.set_location("wp1", aux2));
  EXPECT_TRUE(map.get_location("wp1", aux3));
  EXPECT_EQ(aux2.position.x, aux3.position.x);
  EXPECT_EQ(aux2.position.y, aux3.position.y);
  EXPECT_EQ(aux2.position.z, aux3.position.z);
  EXPECT_EQ(aux2.orientation.x, aux3.orientation.x);
  EXPECT_EQ(aux2.orientation.y, aux3.orientation.y);
  EXPECT_EQ(aux2.orientation.z, aux3.orientation.z);
  EXPECT_EQ(aux2.orientation.w, aux3.orientation.w);

  geometry_msgs::Pose aux4 = map.stringToPose("1.0, 0.0, 2.0");
  EXPECT_EQ(aux2.position.x, aux4.position.x);
  EXPECT_EQ(aux2.position.y, aux4.position.y);
  EXPECT_EQ(aux2.position.z, aux4.position.z);
  EXPECT_EQ(aux2.orientation.x, aux4.orientation.x);
  EXPECT_EQ(aux2.orientation.y, aux4.orientation.y);
  EXPECT_EQ(aux2.orientation.z, aux4.orientation.z);
  EXPECT_EQ(aux2.orientation.w, aux4.orientation.w);
}

TEST(TESTSuite, test_pddl1)
{
  std::string command = "rosrun rosplan_planning_system popf /tmp/domain_topological_navigation.pddl ";
  command = command + ros::package::getPath("topological_navigation") + "/test/problem_test1.pddl > /tmp/out_test1.out";
  EXPECT_EQ(system(command.c_str()), 0);

  std::string comp_command = "diff  /tmp/out_test1.out ";
  comp_command = comp_command + ros::package::getPath("topological_navigation") + "/test/expected_out_test1.out";

  EXPECT_EQ(system(comp_command.c_str()), 0);
}

TEST(TESTSuite, test_pddl2)
{
  std::string command = "rosrun rosplan_planning_system popf /tmp/domain_topological_navigation.pddl ";
  command = command + ros::package::getPath("topological_navigation") + "/test/problem_test2.pddl > /tmp/out_test2.out";
  EXPECT_EQ(system(command.c_str()), 0);

  std::string comp_command = "diff  /tmp/out_test2.out ";
  comp_command = comp_command + ros::package::getPath("topological_navigation") + "/test/expected_out_test2.out";

  EXPECT_EQ(system(comp_command.c_str()), 0);
}

TEST(TESTSuite, test_pddl3)
{
  std::string command = "rosrun rosplan_planning_system popf /tmp/domain_topological_navigation.pddl ";
  command = command + ros::package::getPath("topological_navigation") + "/test/problem_test3.pddl > /tmp/out_test3.out";
  EXPECT_EQ(system(command.c_str()), 0);

  std::string comp_command = "diff  /tmp/out_test3.out ";
  comp_command = comp_command + ros::package::getPath("topological_navigation") + "/test/expected_out_test3.out";

  EXPECT_EQ(system(comp_command.c_str()), 0);
}

TEST(TESTSuite, test_pddl4)
{
  std::string command = "rosrun rosplan_planning_system popf /tmp/domain_topological_navigation.pddl ";
  command = command + ros::package::getPath("topological_navigation") + "/test/problem_test4.pddl > /tmp/out_test4.out";
  EXPECT_EQ(system(command.c_str()), 0);

  std::string comp_command = "diff  /tmp/out_test4.out ";
  comp_command = comp_command + ros::package::getPath("topological_navigation") + "/test/expected_out_test4.out";

  EXPECT_EQ(system(comp_command.c_str()), 0);
}

TEST(TESTSuite, test_pddl5)
{
  std::string command = "rosrun rosplan_planning_system popf /tmp/domain_topological_navigation.pddl ";
  command = command + ros::package::getPath("topological_navigation") + "/test/problem_test5.pddl > /tmp/out_test5.out";
  EXPECT_EQ(system(command.c_str()), 0);

  std::string comp_command = "diff  /tmp/out_test5.out ";
  comp_command = comp_command + ros::package::getPath("topological_navigation") + "/test/expected_out_test5.out";

  EXPECT_EQ(system(comp_command.c_str()), 0);
}

TEST(TESTSuite, test_services)
{
  ros::NodeHandle nh;
  ros::ServiceClient client_set =
      nh.serviceClient<topological_navigation_msgs::SetLocation>("/topological_navigation/set_location");
  ros::ServiceClient client_get =
      nh.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");

  EXPECT_TRUE(ros::service::waitForService("/topological_navigation/set_location", ros::Duration(2)));
  EXPECT_TRUE(ros::service::waitForService("/topological_navigation/get_location", ros::Duration(2)));

  geometry_msgs::Pose pose;
  pose.orientation = tf2::createQuaternionMsgFromYaw(2.0);
  pose.position.x = 1.0;

  topological_navigation_msgs::SetLocation srv_set;
  srv_set.request.waypoint = "wp1";
  srv_set.request.position = pose;

  EXPECT_TRUE(client_set.call(srv_set));

  topological_navigation_msgs::GetLocation srv_get;
  srv_get.request.waypoint = "wp1";

  EXPECT_TRUE(client_get.call(srv_get));

  geometry_msgs::Pose pose2 = srv_get.response.position;

  EXPECT_EQ(pose.position.x, pose2.position.x);
  EXPECT_EQ(pose.position.y, pose2.position.y);
  EXPECT_EQ(pose.position.z, pose2.position.z);
  EXPECT_EQ(pose.orientation.x, pose2.orientation.x);
  EXPECT_EQ(pose.orientation.y, pose2.orientation.y);
  EXPECT_EQ(pose.orientation.z, pose2.orientation.z);
  EXPECT_EQ(pose.orientation.w, pose2.orientation.w);
}

TEST(TESTSuite, test_load_map1)
{
  ros::NodeHandle nh;
  ros::ServiceClient client_get =
      nh.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  topological_navigation_msgs::GetLocation srv_get;
  srv_get.request.waypoint = "wp_corridor_from_maindoor";

  EXPECT_TRUE(client_get.call(srv_get));

  topological_navigation::TopologicalNav map;

  geometry_msgs::Pose aux = srv_get.response.position;
  geometry_msgs::Pose aux2 = map.stringToPose("8.5, 11.5, -1.5707");
  EXPECT_EQ(aux2.position.x, aux.position.x);
  EXPECT_EQ(aux2.position.y, aux.position.y);
  EXPECT_EQ(aux2.position.z, aux.position.z);
  EXPECT_EQ(aux2.orientation.x, aux.orientation.x);
  EXPECT_EQ(aux2.orientation.y, aux.orientation.y);
  EXPECT_EQ(aux2.orientation.z, aux.orientation.z);
  EXPECT_EQ(aux2.orientation.w, aux.orientation.w);
}

class TestKMS : public bica_planning::KMSClient
{
public:
  TestKMS()
  {
  }
  std::vector<std::string> test_get_predicates(std::regex re)
  {
    return search_predicates_regex(re);
  }
  std::vector<std::string> test_get_instances(const std::string& instance_type)
  {
    return get_instances(instance_type);
  }
};

TEST(TESTSuite, test_load_map2)
{
  TestKMS testkms;

  std::vector<std::string> res = testkms.test_get_instances("room");
  EXPECT_EQ(res.size(), 6);
  std::vector<std::string> res2 = testkms.test_get_instances("waypoint");
  EXPECT_EQ(res2.size(), 13);
  std::vector<std::string> res3 = testkms.test_get_instances("door");
  EXPECT_EQ(res3.size(), 4);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_TopologicalNav");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
