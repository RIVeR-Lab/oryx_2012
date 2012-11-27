/*
 * NetworkManagerUI.cpp
 *
 *  Created on: Nov 18, 2012
 *      Author: mitchell
 */

#include "ros/ros.h"
#include "oryx_network_manager/GetNetworkState.h"
#include <sstream>

#include <QApplication>
#include "qmainwindow.h"
#include "qmenubar.h"
#include "qaction.h"
#include "qtextedit.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "oryx_network_manager_ui");
  ros::NodeHandle n;

  //QApplication app(argc, argv);


  ROS_INFO("Getting Network Info");
  oryx_network_manager::GetNetworkState networkState;
  if (ros::service::call("get_network_state", networkState))
  {
    for (unsigned int i = 0; i < networkState.response.connections.size(); ++i)
    {
      oryx_network_manager::NetworkConnection connection = networkState.response.connections[i];
      ROS_INFO(
          "Got Network Connection, %s (%s): %d", connection.name.c_str(), connection.uuid.c_str(), connection.connection_type);
    }
    for (unsigned int i = 0; i < networkState.response.devices.size(); ++i)
    {
      oryx_network_manager::NetworkDevice device = networkState.response.devices[i];
      ROS_INFO("Got Network Device, %s (%s): %d", device.iface.c_str(), device.product.c_str(), device.device_type);
    }
  }
  ROS_INFO("Got Network Info");

  /*QMainWindow* window = new QMainWindow();
  window->setWindowTitle("Hello Qt");
  QMenu* fileMenu = window->menuBar()->addMenu("File");
  fileMenu->addSeparator();
  QAction* exitAction = new QAction("Exit", window);
  QObject::connect(exitAction, SIGNAL(triggered()), qApp, SLOT(quit()));
  window->setCentralWidget(new QTextEdit());
  window->show();

  return app.exec();*/

  return 0;
}

