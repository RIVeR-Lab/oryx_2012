/*
 * NetworkConnectionManager.h
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 *
 * A class which handles changes in the connections and provides a service to retrieve the connections
 */

#ifndef NETWORKCONNECTIONMANAGER_H_
#define NETWORKCONNECTIONMANAGER_H_

#include "ros/ros.h"
#include "oryx_network_manager/NetworkConnections.h"
#include <nm-remote-settings.h>
#include <map>

using namespace oryx_network_manager;

class NetworkConnectionManager
{
public:
  NetworkConnectionManager(ros::NodeHandle& handle, NMRemoteSettings* remote_settings);
  virtual ~NetworkConnectionManager();
private:
  void process_added_connection(NMConnection* connection);
  void process_removed_connection(NMConnection* connection);
  void process_updated_connection(NMConnection* connection);
  void fill_connection_info(NMConnection* connection, oryx_network_manager::NetworkConnection& ros_connection_message);
  void publish_connections();

  static void connection_added_cb(NMRemoteSettings* settings, NMRemoteConnection* connection, gpointer user_data);
  static void initial_connections_read_cb(NMRemoteSettings* settings, gpointer user_data);
  static void connection_removed_cb(NMRemoteConnection *connection, gpointer user_data);
  static void connection_updated_cb(NMRemoteConnection *connection, gpointer user_data);
private:
  ros::NodeHandle& m_handle;
  NMRemoteSettings* m_remote_settings;
  uint32_t msg_seq;
  ros::Publisher connection_pub;
  std::map<std::string, NetworkConnection> connections;
};

#endif /* NETWORKCONNECTIONMANAGER_H_ */
