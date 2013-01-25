/*
 * NetworkConfigurationManager.cpp
 *
 *  Created on: Nov 29, 2012
 *      Author: mitchell
 */

#include "NetworkConfigurationManager.h"
#include <stdio.h>

NetworkConfigurationManager::NetworkConfigurationManager(ros::NodeHandle& handle, NMClient* client,
    NMRemoteSettings* remote_settings) :
    m_handle(handle), m_client(client), m_remote_settings(remote_settings), connection_sub(
        handle.subscribe<NetworkConnections>("oryx_network_connections", 10,
            boost::bind(&NetworkConfigurationManager::connections_changed_cb, this, _1))), active_connection_sub(
        handle.subscribe<ActiveNetworkConnections>("oryx_active_network_connections", 10,
            boost::bind(&NetworkConfigurationManager::active_connections_changed_cb, this, _1))), m_config(
        new BlankNetworkConfig())
{
}

void NetworkConfigurationManager::load_config_from_file(const char* filename)
{
  ROS_INFO("Loading Network Configuration File: %s", filename);
  std::vector<NetworkConnectionConfig> configs;
  FILE * pFile = fopen(filename, "r");
  if (!pFile)
  {
    ROS_WARN("Error reading config file: %s", filename);
    return;
  }
  char name[100];
  int num_read;
  num_read = fscanf(pFile, "%[^\n\r]", name);
  if (num_read != 1)
  {
    ROS_WARN("Badly formatted config file: %s", filename);
  }
  else
  {
    char iface[100];
    char connection_name[100];
    do
    {
      num_read = fscanf(pFile, "%*[ \n\t]%[^,],%[^\n\r]", iface, connection_name);
      if (num_read == 2)
      {
        NetworkConnectionConfig config(iface, connection_name);
        configs.push_back(config);
      }
    } while (num_read == 2);
    if (num_read > 0)
    {
      ROS_WARN("Badly formatted config file: %s. only read %i items on connection line", filename, num_read);
    }
    else
    {
      boost::shared_ptr<NetworkConfig> new_config(new NetworkConfig(name, configs));
      set_config(new_config);
    }

  }

  fclose(pFile);
}

void NetworkConfigurationManager::set_config(boost::shared_ptr<NetworkConfig> new_config)
{
  m_config = new_config;
  ROS_INFO("Applying Network Configuration: %s", m_config->get_name().c_str());
  update_config();
}

void NetworkConfigurationManager::active_connections_changed_cb(
    const ActiveNetworkConnections::ConstPtr& latest_active_connections)
{
  m_latest_active_connections = latest_active_connections;
  update_config();
}
void NetworkConfigurationManager::connections_changed_cb(const NetworkConnections::ConstPtr& latest_connections)
{
  m_latest_connections = latest_connections;
  update_config();
}
void NetworkConfigurationManager::update_config()
{
  ROS_DEBUG("Cheching Network Config");
  const NetworkConnections::ConstPtr connections = m_latest_connections;
  const ActiveNetworkConnections::ConstPtr active_connections = m_latest_active_connections;
  if (!connections || !active_connections) //haven't yet received any connection data
    return;

  if (m_config)
  {
    m_config->update(active_connections, this);
  }

}

void NetworkConfig::update(const ActiveNetworkConnections::ConstPtr active_connections, NetworkConfigurationManager* manager) const
{
  std::vector<NetworkConnectionConfig>::const_iterator connections_itr = m_connection_configs.begin();
  for (; connections_itr != m_connection_configs.end(); ++connections_itr)
    connections_itr->update(active_connections, manager);
}
void NetworkConnectionConfig::update(const ActiveNetworkConnections::ConstPtr active_connections, NetworkConfigurationManager* manager) const
{
  bool found = false;
  std::vector<ActiveNetworkConnection>::const_iterator connection_itr = active_connections->list.begin();
  for (; connection_itr != active_connections->list.end(); ++connection_itr)
  {
    if (connection_itr->connection.name == m_connection_name && connection_itr->device.iface == m_iface)
      found = true;
  }
  if (!found)
  {
    ROS_INFO("Not connected to %s on %s. Attempting to connect now", m_connection_name.c_str(), m_iface.c_str());
    if (!manager->activate_connection(m_iface, m_connection_name))
      ROS_WARN("Failed to connect to %s on %s", m_connection_name.c_str(), m_iface.c_str());
  }
}

bool NetworkConfigurationManager::activate_connection(std::string iface, std::string connection_name)
{
  const NetworkConnections::ConstPtr connections = m_latest_connections;
  if (!connections) //haven't yet received any connection data
    return false;

  NMDevice * device = nm_client_get_device_by_iface(m_client, iface.c_str());
  if (device == NULL)
  {
    ROS_WARN("Could not find device: '%s'", iface.c_str());
    return false;
  }

  //get the connection info to connect to
  const char* path = NULL;
  std::vector<NetworkConnection>::const_iterator itr = connections->list.begin();
  for (; itr != connections->list.end(); ++itr)
  {
    if (itr->name == connection_name)
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
    ROS_WARN( "Could not find connection: '%s'", connection_name.c_str());
    return false;
  }

  ROS_INFO( "Activiting Connection: '%s' on %s", connection_name.c_str(), iface.c_str());

  nm_client_activate_connection(m_client, connection, device, NULL, NULL, NULL);

  sleep(1);//block for a second to allow for connection activation to begin

  return true;
}

NetworkConfigurationManager::~NetworkConfigurationManager()
{
}

