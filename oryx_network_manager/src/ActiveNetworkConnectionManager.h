/*
 * ActiveNetworkConnectionManager.h
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 *
 * A class which handles changes in the active connections and provides a service to retrieve the active connections
 */

#ifndef ACTIVENETWORKCONNECTIONMANAGER_H_
#define ACTIVENETWORKCONNECTIONMANAGER_H_

#include "ros/ros.h"
#include <nm-client.h>
#include "oryx_network_manager/NetworkConnections.h"
#include "oryx_network_manager/NetworkDevices.h"
#include "oryx_network_manager/ActiveNetworkConnections.h"

using namespace oryx_network_manager;

class ActiveNetworkConnectionManager
{
public:
  ActiveNetworkConnectionManager(ros::NodeHandle& handle, NMClient* client);
  virtual ~ActiveNetworkConnectionManager();
private:
  void process_active_connections_changed();
  void connections_changed_cb(const NetworkConnections::ConstPtr& latest_connections);
  void devices_changed_cb(const NetworkDevices::ConstPtr& latest_devices);
  static void active_connections_changed_cb (NMClient *client, GParamSpec *pspec, gpointer user_data);
private:
  ros::NodeHandle& m_handle;
  NMClient* m_client;
  uint32_t msg_seq;
  ros::Subscriber connection_sub;
  ros::Subscriber devices_sub;
  ros::Publisher active_connection_pub;
  NetworkConnections::ConstPtr m_latest_connections;
  NetworkDevices::ConstPtr m_latest_devices;
};

#endif /* ACTIVENETWORKCONNECTIONMANAGER_H_ */
