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
#include "tm_creator/TMCreator.h"

namespace tm_creator
{

TMCreator::TMCreator()
: rqt_gui_cpp::Plugin()
, widget_(0)
, finished_(false)
{
  setObjectName("TMCreator");
  add_wp_pub_ = n_.advertise<AddWp>("/tm_creator/add_wp_marker", 1);
  rm_wp_pub_ = n_.advertise<std_msgs::Empty>("/tm_creator/rm_marker", 1);
  add_door_pub_ = n_.advertise<AddDoor>("/tm_creator/add_door_marker", 1);
  create_link_pub_ = n_.advertise<std_msgs::Empty>("/tm_creator/create_link", 1);
  save_client_ = n_.serviceClient<SaveMap>("/tm_creator/save_map");
  load_client_ = n_.serviceClient<SaveMap>("/tm_creator/load_map");
}

void TMCreator::initPlugin(qt_gui_cpp::PluginContext& context)
{
  QStringList argv = context.argv();
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  connect(ui_.wpAddButton, &QPushButton::clicked, this, &TMCreator::addWp);
  connect(ui_.rmButton, &QPushButton::clicked, this, &TMCreator::rm);
  connect(ui_.doorAddButton, &QPushButton::clicked, this, &TMCreator::addDoor);
  connect(ui_.linkButton, &QPushButton::clicked, this, &TMCreator::link);
  connect(ui_.saveButton, &QPushButton::clicked, this, &TMCreator::save);
  connect(ui_.loadButton, &QPushButton::clicked, this, &TMCreator::load);
  context.addWidget(widget_);
}

void TMCreator::addWp()
{
  AddWp msg;
  msg.wp_name = ui_.wpName->toPlainText().toUtf8().constData();
  msg.room = ui_.wpRoom->toPlainText().toUtf8().constData();
  if (msg.room == "")
  {
    QMessageBox msgBox;
    msgBox.setText("You must define in which room will be the wp");
    msgBox.exec();
  }
  else
    add_wp_pub_.publish(msg);
}

void TMCreator::rm()
{
  std_msgs::Empty msg;
  rm_wp_pub_.publish(msg);
}

void TMCreator::save()
{
  QString dir_name = QFileDialog::getExistingDirectory(
    widget_,
    tr("Select dir"),
    "/home",
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  SaveMap srv;
  srv.request.path = dir_name.toUtf8().constData();;
  if (save_client_.call(srv))
  {
    QMessageBox msgBox;
    msgBox.setText("Saved!");
    msgBox.exec();
  }
}

void TMCreator::load()
{
  QString selfilter = tr("YAML (*.yaml)");
  QString file_name = QFileDialog::getOpenFileName(
    widget_,
    tr("Select topological_map"),
    "/home",
    tr("YAML (*.yaml)"),
    &selfilter);
  SaveMap srv;
  srv.request.path = file_name.toUtf8().constData();;
  if (load_client_.call(srv))
  {
    QMessageBox msgBox;
    msgBox.setText("Loaded!");
    msgBox.exec();
  }
}

void TMCreator::addDoor()
{
  AddDoor msg;
  msg.room_1 = ui_.doorName->toPlainText().toUtf8().constData();
  msg.room_2 = ui_.doorName_2->toPlainText().toUtf8().constData();
  if (msg.room_1 == "" || msg.room_2 == "")
  {
    QMessageBox msgBox;
    msgBox.setText("You must define in which rooms will be the door");
    msgBox.exec();
  }
  else
    add_door_pub_.publish(msg);
}

void TMCreator::link()
{
  std_msgs::Empty msg;
  create_link_pub_.publish(msg);
}

void TMCreator::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void TMCreator::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

void TMCreator::shutdownPlugin()
{
  finished_ = true;
}

}  //  namespace tm_creator
PLUGINLIB_DECLARE_CLASS(tm_creator, TMCreator, tm_creator::TMCreator, rqt_gui_cpp::Plugin)
