/*
 * ActiveNetworkConnectionManager.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 */

#include "ActiveNetworkConnectionManager.h"
#include <boost/bind.hpp>
#include "MessageUtil.h"

ActiveNetworkConnectionManager::ActiveNetworkConnectionManager(ros::NodeHandle& handle, NMClient* client) :
m_handle(handle), m_client(client), msg_seq(0), connection_sub(
    handle.subscribe<NetworkConnections>("oryx_network_connections", 10,
        boost::bind(&ActiveNetworkConnectionManager::connections_changed_cb, this, _1))), devices_sub(
            handle.subscribe<NetworkDevices>("oryx_network_devices", 10,
                boost::bind(&ActiveNetworkConnectionManager::devices_changed_cb, this, _1))), active_connection_pub(
                    handle.advertise<ActiveNetworkConnections>("oryx_active_network_connections", 10, true))
{
  process_active_connections_changed();
  g_signal_connect(client, "notify::active-connections", G_CALLBACK(active_connections_changed_cb), this);
}

void ActiveNetworkConnectionManager::connections_changed_cb(const NetworkConnections::ConstPtr& latest_connections)
{
  m_latest_connections = latest_connections;
  process_active_connections_changed();
}
void ActiveNetworkConnectionManager::devices_changed_cb(const NetworkDevices::ConstPtr& latest_devices)
{
  m_latest_devices = latest_devices;
  process_active_connections_changed();
}

void ActiveNetworkConnectionManager::process_active_connections_changed()
{
  ROS_DEBUG("Updating Active Network Connections");
  const NetworkConnections::ConstPtr connections = m_latest_connections;
  const NetworkDevices::ConstPtr devices = m_latest_devices;
  if(!connections || !devices)//haven't yet received any device and connection data
    return;

  oryx_network_manager::ActiveNetworkConnections ros_active_connections_message;

  const GPtrArray *active_list = nm_client_get_active_connections(m_client);
  for (unsigned int i = 0; active_list && (i < active_list->len); i++)
  {
    NMActiveConnection* active_connection = NM_ACTIVE_CONNECTION(g_ptr_array_index(active_list, i));
    oryx_network_manager::ActiveNetworkConnection ros_active_connection_message;

    if (nm_active_connection_get_state(active_connection) == NM_ACTIVE_CONNECTION_STATE_ACTIVATED)
      ros_active_connection_message.connection_state = oryx_network_manager::ActiveNetworkConnection::ACTIVATED_STATE;
    else if (nm_active_connection_get_state(active_connection) == NM_ACTIVE_CONNECTION_STATE_ACTIVATING)
      ros_active_connection_message.connection_state = oryx_network_manager::ActiveNetworkConnection::ACTIVATING_STATE;
    else if (nm_active_connection_get_state(active_connection) == NM_ACTIVE_CONNECTION_STATE_DEACTIVATING)
      ros_active_connection_message.connection_state =
          oryx_network_manager::ActiveNetworkConnection::DEACTIVATING_STATE;
    else if (nm_active_connection_get_state(active_connection) == NM_ACTIVE_CONNECTION_STATE_UNKNOWN)
      ros_active_connection_message.connection_state = oryx_network_manager::ActiveNetworkConnection::UNKNOWN_STATE;

    const char* active_connection_path = nm_active_connection_get_connection(active_connection);

    if(!active_connection_path){
      ROS_WARN("Active Network Connections: path of an active connection returned NULL?");
      continue;
    }
    //get the connection info for this active connection
    std::vector<NetworkConnection>::const_iterator itr = connections->list.begin();
    for (; itr != connections->list.end(); ++itr)
    {
      if (itr->path == active_connection_path)
      {
        ros_active_connection_message.connection = *itr;
        break;
      }
    }

    //if there is a device retrieve the info about it
    const GPtrArray *active_connection_devices_devices = nm_active_connection_get_devices(active_connection);
    if (active_connection_devices_devices && active_connection_devices_devices->len > 0)
    {
      NMDevice* device = (NMDevice*) g_ptr_array_index (active_connection_devices_devices, 0);
      const char* iface = nm_device_get_iface(device);

      //get the device info for this active connection
      std::vector<NetworkDevice>::const_iterator itr = devices->list.begin();
      for (; itr != devices->list.end(); ++itr)
      {
        if (itr->iface == iface)
        {
          ros_active_connection_message.device = *itr;
          break;
        }
      }

      //get ip information if it's available
      NMIP4Config* ip4_config = nm_device_get_ip4_config(device);
      if(ip4_config)
      {
        const GSList* addresses = nm_ip4_config_get_addresses(ip4_config);
        if (g_slist_length((GSList *) addresses) && addresses->data)
        {
          char ip_address_str[20];
          guint32 ip_address = nm_ip4_address_get_address((NMIP4Address*)addresses->data);
          sprintf(ip_address_str, "%d.%d.%d.%d", (ip_address>>0)&0xFF, (ip_address>>8)&0xFF, (ip_address>>16)&0xFF, (ip_address>>24)&0xFF);
          ros_active_connection_message.ipv4_address = ip_address_str;
        }
      }
    }
    ros_active_connections_message.list.push_back(ros_active_connection_message);
  }
  fill_header_message(ros_active_connections_message.header, msg_seq++);
  active_connection_pub.publish(ros_active_connections_message);
}

void ActiveNetworkConnectionManager::active_connections_changed_cb(NMClient *client, GParamSpec *pspec,
    gpointer user_data)
{
  ActiveNetworkConnectionManager* manager = (ActiveNetworkConnectionManager*) user_data;
  manager->process_active_connections_changed();
}

ActiveNetworkConnectionManager::~ActiveNetworkConnectionManager()
{
}

