/*
 * ActivateConnectionService.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 */

#include "ActivateConnectionService.h"
#include "boost/bind.hpp"

ActivateConnectionService::ActivateConnectionService(ros::NodeHandle& handle, NMClient* client,
    NMRemoteSettings* remote_settings) :
    m_handle(handle), m_client(client), m_remote_settings(remote_settings), connection_sub(
        handle.subscribe<NetworkConnections>("oryx_network_connections", 10,
            boost::bind(&ActivateConnectionService::connections_changed_cb, this, _1))), activate_service(
                handle.advertiseService<ActivateConnection::Request, ActivateConnection::Response>("oryx_activate_network_connection",
                    boost::bind(&ActivateConnectionService::activate_connection_cb, this, _1, _2)))
{
}

void ActivateConnectionService::connections_changed_cb(const NetworkConnections::ConstPtr& latest_connections)
{
  m_latest_connections = latest_connections;
}

bool ActivateConnectionService::activate_connection_cb(ActivateConnection::Request& request,
    ActivateConnection::Response& response)
{
  const NetworkConnections::ConstPtr connections = m_latest_connections;
  if (!connections) //haven't yet received any connection data
    return false;

  NMDevice * device = nm_client_get_device_by_iface(m_client, request.device_iface.c_str());
  if (device == NULL)
  {
    ROS_WARN("Could not find device: '%s'", request.device_iface.c_str());
    response.message = "Could not find device: '";
    response.message += request.device_iface;
    response.message += "'";
    return true;
  }
  //get the connection info for this active connection
  const char* path = NULL;
  std::vector<NetworkConnection>::const_iterator itr = connections->list.begin();
  for (; itr != connections->list.end(); ++itr)
  {
    if (itr->name == request.connection_name)
    {
      path = itr->path.c_str();
      break;
    }
  }
  NMConnection * connection = NULL;
  if (path != NULL)
    connection = NM_CONNECTION(nm_remote_settings_get_connection_by_path(m_remote_settings, path));
  if (connection == NULL)
  {
    ROS_WARN( "Could not find connection: '%s'", request.connection_name.c_str());
    response.message = "Could not find connection: '";
    response.message += request.connection_name;
    response.message += "'";
    return true;
  }

  ROS_INFO( "Activiting Connection: '%s' on %s", request.connection_name.c_str(), request.device_iface.c_str());

  nm_client_activate_connection(m_client, connection, device, NULL, NULL, NULL);

  return true;
}

ActivateConnectionService::~ActivateConnectionService()
{
}

