/*
 * NetworkConfigurationManager.h
 *
 *  Created on: Nov 29, 2012
 *      Author: mitchell
 *
 */

#ifndef NETWORKCONFIGURATIONMANAGER_H_
#define NETWORKCONFIGURATIONMANAGER_H_

#include "oryx_network_manager/ActiveNetworkConnections.h"
#include "oryx_network_manager/NetworkConnections.h"
#include "boost/shared_ptr.hpp"
#include "ros/ros.h"
#include <nm-client.h>
#include <nm-remote-settings.h>
#include <vector>
using namespace oryx_network_manager;

class NetworkConfigurationManager;

/**
 * A configuration which outlines how a connection will be connected
 */
class NetworkConnectionConfig{
public:
  NetworkConnectionConfig(std::string iface, std::string connection_name):m_iface(iface), m_connection_name(connection_name){};
  virtual ~NetworkConnectionConfig(){};
  virtual void update(const ActiveNetworkConnections::ConstPtr active_connections, NetworkConfigurationManager* manager)const;
private:
  std::string m_iface;
  std::string m_connection_name;
};
/**
 * A class which represents a system network configuration
 */
class NetworkConfig{
public:
  NetworkConfig(std::string name, std::vector<NetworkConnectionConfig> connection_configs):m_name(name), m_connection_configs(connection_configs){};
  virtual ~NetworkConfig(){};
  virtual void update(const ActiveNetworkConnections::ConstPtr active_connections, NetworkConfigurationManager* manager) const;
  const std::string get_name(){return m_name;};
private:
  std::string m_name;
  std::vector<NetworkConnectionConfig> m_connection_configs;
};

/**
 * A system network configuration which does nothing
 */
class BlankNetworkConfig : public NetworkConfig{
public:
  BlankNetworkConfig():NetworkConfig("None", std::vector<NetworkConnectionConfig>()){};
  virtual ~BlankNetworkConfig(){};
  virtual void update(const ActiveNetworkConnections::ConstPtr active_connections, NetworkConfigurationManager* manager)const{};
};

/**
 * A class which manages the the active configuration
 */
class NetworkConfigurationManager
{
public:
  NetworkConfigurationManager(ros::NodeHandle& handle, NMClient* client, NMRemoteSettings* remote_settings);
  virtual ~NetworkConfigurationManager();
  void load_config_from_file(const char* filename);
  void set_config(boost::shared_ptr<NetworkConfig> new_config);
  bool activate_connection(std::string iface, std::string connection_name);
private:
  void active_connections_changed_cb(const ActiveNetworkConnections::ConstPtr& latest_connections);
  void connections_changed_cb(const NetworkConnections::ConstPtr& latest_connections);
  void update_config();
private:
  ros::NodeHandle& m_handle;
  NMClient* m_client;
  NMRemoteSettings* m_remote_settings;
  ros::Subscriber connection_sub;
  NetworkConnections::ConstPtr m_latest_connections;
  ros::Subscriber active_connection_sub;
  ActiveNetworkConnections::ConstPtr m_latest_active_connections;
  boost::shared_ptr<NetworkConfig> m_config;
};


#endif /* NETWORKCONFIGURATIONMANAGER_H_ */
