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
#ifndef TMCREATOR_H
#define TMCREATOR_H

#include <rqt_gui_cpp/plugin.h>
#include <topological_map_creator/ui_tm_creator.h>
#include <QWidget>
#include <QPoint>
#include <QMenu>
#include <QInputDialog>
#include <QMessageBox>
#include <QString>
#include <QFileDialog>
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <topological_map_creator_msgs/AddDoor.h>
#include <topological_map_creator_msgs/AddWp.h>
#include <topological_map_creator_msgs/SaveMap.h>

using topological_map_creator_msgs::AddWp;
using topological_map_creator_msgs::AddDoor;
using topological_map_creator_msgs::SaveMap;

namespace tm_creator
{

class TMCreator
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  TMCreator();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
     qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
     const qt_gui_cpp::Settings& instance_settings);
  void loop();
  public slots:
    void addWp();
    void rm();
    void save();
    void load();
    void addDoor();
    void link();

private:
  Ui::TMCreator ui_;
  QWidget* widget_;
  bool finished_;
  ros::NodeHandle n_;
  ros::Publisher add_wp_pub_, rm_wp_pub_, add_door_pub_, rm_door_pub_,
    create_link_pub_;
  ros::ServiceClient save_client_, load_client_;
};
}  //  namespace tm_creator
#endif  // TMCREATOR_H
