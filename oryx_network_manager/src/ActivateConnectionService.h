/*
 * ActivateConnectionService.h
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 */

#ifndef ACTIVATECONNECTIONSERVICE_H_
#define ACTIVATECONNECTIONSERVICE_H_

#include "ros/ros.h"
#include "nm-client.h"
#include <nm-remote-settings.h>

#include "oryx_network_manager/ActivateConnection.h"
#include "oryx_network_manager/NetworkConnections.h"

using namespace oryx_network_manager;

class ActivateConnectionService
{
public:
  ActivateConnectionService(ros::NodeHandle& handle, NMClient* client, NMRemoteSettings* remote_settings);
  virtual ~ActivateConnectionService();

private:
  bool activate_connection_cb(ActivateConnection::Request& request, ActivateConnection::Response& response);
  void connections_changed_cb(const NetworkConnections::ConstPtr& latest_connections);
private:
  ros::NodeHandle& m_handle;
  NMClient* m_client;
  NMRemoteSettings* m_remote_settings;
  ros::Subscriber connection_sub;
  NetworkConnections::ConstPtr m_latest_connections;
  ros::ServiceServer activate_service;
};

#endif /* ACTIVATECONNECTIONSERVICE_H_ */
