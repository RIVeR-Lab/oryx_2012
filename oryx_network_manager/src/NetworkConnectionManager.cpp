/*
 * NetworkConnectionManager.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 */

#include "NetworkConnectionManager.h"
#include <nm-connection.h>
#include <nm-setting-connection.h>
#include <NetworkManager.h>
#include <nm-utils.h>
#include "MessageUtil.h"

NetworkConnectionManager::NetworkConnectionManager(ros::NodeHandle& handle, NMRemoteSettings* remote_settings) :
    m_handle(handle), m_remote_settings(remote_settings), msg_seq(0), connection_pub(
        handle.advertise<NetworkConnections>("oryx_network_connections", 10, true))
{
  ROS_INFO("Initing Network Connection Manager");
  publish_connections();
  g_signal_connect(m_remote_settings, NM_REMOTE_SETTINGS_NEW_CONNECTION, G_CALLBACK(connection_added_cb), this);
}

void NetworkConnectionManager::process_added_connection(NMConnection *connection)
{
  g_signal_connect(connection, NM_REMOTE_CONNECTION_UPDATED, G_CALLBACK(connection_updated_cb), this);
  g_signal_connect(connection, NM_REMOTE_CONNECTION_REMOVED, G_CALLBACK(connection_removed_cb), this);
  ROS_INFO("Network Connection Added: %s", nm_connection_get_path(connection));

  NetworkConnection connection_info;
  fill_connection_info(connection, connection_info);
  connections[nm_connection_get_path(connection)] = connection_info;

  publish_connections();
}
void NetworkConnectionManager::process_removed_connection(NMConnection *connection)
{
  //g_signal_handlers_disconnect_by_func(connection, (gpointer)G_CALLBACK (connection_removed_cb), this);
  //g_signal_handlers_disconnect_by_func(connection, (gpointer)G_CALLBACK (connection_updated_cb), this);
  ROS_INFO("Network Connection Removed: %s", nm_connection_get_path(connection));
  connections.erase(nm_connection_get_path(connection));

  publish_connections();
}
void NetworkConnectionManager::process_updated_connection(NMConnection* connection)
{
  ROS_INFO("Network Connection Updated: %s", nm_connection_get_path(connection));
  NetworkConnection connection_info;
  fill_connection_info(connection, connection_info);
  connections[nm_connection_get_path(connection)] = connection_info;

  publish_connections();
}


void NetworkConnectionManager::fill_connection_info(NMConnection* connection,
    oryx_network_manager::NetworkConnection& ros_connection_message)
{
  ros_connection_message.path = nm_connection_get_path(connection);
  NMSettingConnection* connection_setting = nm_connection_get_setting_connection(connection);
  if (connection_setting)
  {
    ros_connection_message.name = nm_setting_connection_get_id(connection_setting);

    if (nm_connection_is_type(connection, NM_SETTING_WIRELESS_SETTING_NAME))
    {
      ros_connection_message.connection_type = oryx_network_manager::NetworkConnection::WIFI_CONNECTION;

      NMSettingWireless *connection_wifi_setting = nm_connection_get_setting_wireless(connection);
      NMSettingWirelessSecurity *connection_wifi_security_setting = nm_connection_get_setting_wireless_security(
          connection);
      //NMSetting8021x *connection_8021x_setting = nm_connection_get_setting_802_1x(connection_data);

      if (connection_wifi_setting != NULL)
      {
        char* ssidTmp = nm_utils_ssid_to_utf8(nm_setting_wireless_get_ssid(connection_wifi_setting));
        ros_connection_message.wifi_ssid = ssidTmp;
        g_free(ssidTmp);

        if (connection_wifi_security_setting)
        {
          const char* key_mgmt = nm_setting_wireless_security_get_key_mgmt(connection_wifi_security_setting);
          const char *auth_alg = nm_setting_wireless_security_get_auth_alg(connection_wifi_security_setting);
          if (!strcmp(key_mgmt, "none"))
            ros_connection_message.wifi_security = "WEP";
          else if (!strcmp(key_mgmt, "wpa-none"))
            ros_connection_message.wifi_security = "WPA/WPA2";
          else if (!strcmp(key_mgmt, "wpa-psk"))
            ros_connection_message.wifi_security = "WPA/WPA2-PSK";
          else if (!strcmp(key_mgmt, "ieee8021x"))
          {
            if (auth_alg && !strcmp(auth_alg, "leap"))
              ros_connection_message.wifi_security = "IEEE 802.1X (LEAP)";
            else
              ros_connection_message.wifi_security = "IEEE 802.1X (Dynamic WEP)";
          }
          else if (!strcmp(key_mgmt, "wpa-eap"))
            ros_connection_message.wifi_security = "WPA/WPA2-EAP";
          else
            ros_connection_message.wifi_security = key_mgmt;
        }
      }
    }
    else if (nm_connection_is_type(connection, NM_SETTING_WIRED_SETTING_NAME))
    {
      ros_connection_message.connection_type = oryx_network_manager::NetworkConnection::ETHERNET_CONNECTION;
    }
    else
    {
      ros_connection_message.connection_type = oryx_network_manager::NetworkConnection::OTHER_CONNECTION;
    }

  }
}
void NetworkConnectionManager::publish_connections()
{
  NetworkConnections connections_message;
  std::map<std::string, NetworkConnection>::iterator itr = connections.begin();
  for (; itr != connections.end(); ++itr)
    connections_message.list.push_back(itr->second);
  fill_header_message(connections_message.header, msg_seq++);
  connection_pub.publish(connections_message);
}

void NetworkConnectionManager::connection_added_cb(NMRemoteSettings *settings, NMRemoteConnection *connection,
    gpointer user_data)
{
  NetworkConnectionManager* manager = (NetworkConnectionManager*) user_data;
  manager->process_added_connection(NM_CONNECTION(connection) );
}
void NetworkConnectionManager::connection_removed_cb(NMRemoteConnection *connection, gpointer user_data)
{
  NetworkConnectionManager* manager = (NetworkConnectionManager*) user_data;
  manager->process_removed_connection(NM_CONNECTION(connection) );
}
void NetworkConnectionManager::connection_updated_cb(NMRemoteConnection *connection, gpointer user_data)
{
  NetworkConnectionManager* manager = (NetworkConnectionManager*) user_data;
  manager->process_updated_connection(NM_CONNECTION(connection) );
}

NetworkConnectionManager::~NetworkConnectionManager()
{
  //g_signal_handlers_disconnect_by_func(m_remote_settings, (gpointer)G_CALLBACK (connection_added_cb), this);
  //g_signal_handlers_disconnect_by_func(m_remote_settings, (gpointer)G_CALLBACK (got_connections_cb), this);
  g_object_unref(m_remote_settings);
}

