#include "ros/ros.h"
#include "oryx_network_manager/NetworkConnection.h"

#include "NetworkManagerUtil.h"

#include <glib.h>
#include <dbus/dbus-glib.h>
#include <dbus/dbus.h>
#include <stdio.h>
#include <stdlib.h>

#include <nm-connection.h>
#include <nm-setting-connection.h>
#include <NetworkManager.h>
#include <nm-utils.h>
#include <nm-client.h>
#include <nm-device.h>
#include <nm-device-wifi.h>
#include <nm-device-ethernet.h>
#include <nm-device-modem.h>
#include <nm-device-wimax.h>

#include "oryx_network_manager/GetNetworkConnections.h"
#include "oryx_network_manager/GetNetworkDevices.h"
#include "oryx_network_manager/GetActiveNetworkConnections.h"
#include "oryx_network_manager/ActivateConnection.h"

#include <pthread.h>

#include "NetworkSettings.h"

#include "NetworkManagerConfig.h"

#define UPDATE_DELAY 15

DBusGConnection *bus;
NMClient *client;
GMainLoop *loop;
pthread_t g_main_loop_thread;

NetworkManagerConfig* config = NULL;

void config_update_callback(const ros::TimerEvent& event)
{
  NetworkManagerConfig* local_config = config;
  //ROS_DEBUG("Updating Configuration...");
  if (local_config)
    local_config->update(bus, client);
}

void set_config(NetworkManagerConfig* _config)
{
  if (config != _config)
  {
    config = _config;
    if (config)
      ROS_INFO("Network Manager switched to '%s' configuration", config->getName().c_str());
    else
      ROS_INFO("Network Manager set to manual mode");
    config_update_callback(ros::TimerEvent());
  }
}

//spawned in new thread to run the glib main loop
static void* run_main_loop(void* userPtr)
{
  g_main_loop_run(loop);
  pthread_exit(NULL);
}

//spawns the glib main loop in a new thread
static void spawn_g_main_loop()
{
  int rc = pthread_create(&g_main_loop_thread, NULL, run_main_loop, NULL);
  if (rc)
  {
    ROS_ERROR("ERROR; return code from pthread_create() is %d\n", rc);
  }
}

static void connection_activation_callback(NMClient *client, NMActiveConnection *active, GError *error,
    gpointer user_data)
{
  if (error)
  { //TODO figure out how to return this message in the service call
    ROS_WARN(
        "%s: (%d) %s", "Connection activation failed", error->code, error->message ? error->message : "Unknown error");
  }
  else
  {
    ROS_INFO("Successfully Activating Connection");
  }
}

//service callback for getting a list of all devices and connections
static bool activate_connection_callback(oryx_network_manager::ActivateConnection::Request& request,
    oryx_network_manager::ActivateConnection::Response& response)
{
  NetworkSettings settings(bus);
  NMDevice * device = get_device_by_iface(client, request.device_iface.c_str());
  if (device == NULL)
  {
    ROS_WARN("Could not find device: '%s'", request.device_iface.c_str());
    response.message = "Could not find device: '";
    response.message += request.device_iface;
    response.message += "'";
    return true;
  }

  NMConnection * connection = get_connection_by_name(settings, request.connection_name.c_str());
  if (connection == NULL)
  {
    ROS_WARN( "Could not find connection: '%s'", request.connection_name.c_str());
    response.message = "Could not find connection: '";
    response.message += request.connection_name;
    response.message += "'";
    return true;
  }

  ROS_INFO( "Activiting Connection: '%s' on %s", request.connection_name.c_str(), request.device_iface.c_str());

  set_config(NULL);
  nm_client_activate_connection(client, connection, device, NULL, connection_activation_callback, NULL);

  return true;
}

static void fill_connection_info(NMConnection* connection_data,
    oryx_network_manager::NetworkConnection* ros_connection_message)
{
  NMSettingConnection *connection_setting = nm_connection_get_setting_connection(connection_data);
  if (connection_setting)
  {
    ros_connection_message->name = nm_setting_connection_get_id(connection_setting);
    ros_connection_message->uuid = nm_setting_connection_get_uuid(connection_setting);

    if (nm_connection_is_type(connection_data, NM_SETTING_WIRELESS_SETTING_NAME))
    {
      ros_connection_message->connection_type = oryx_network_manager::NetworkConnection::WIFI_CONNECTION;

      NMSettingWireless *connection_wifi_setting = nm_connection_get_setting_wireless(connection_data);
      NMSettingWirelessSecurity *connection_wifi_security_setting = nm_connection_get_setting_wireless_security(
          connection_data);
      //NMSetting8021x *connection_8021x_setting = nm_connection_get_setting_802_1x(connection_data);

      if (connection_wifi_setting != NULL)
      {
        char* ssidTmp = nm_utils_ssid_to_utf8(nm_setting_wireless_get_ssid(connection_wifi_setting));
        ros_connection_message->wifi_ssid = ssidTmp;
        g_free(ssidTmp);

        if (connection_wifi_security_setting)
        {
          const char* key_mgmt = nm_setting_wireless_security_get_key_mgmt(connection_wifi_security_setting);
          const char *auth_alg = nm_setting_wireless_security_get_auth_alg(connection_wifi_security_setting);
          if (!strcmp(key_mgmt, "none"))
            ros_connection_message->wifi_security = "WEP";
          else if (!strcmp(key_mgmt, "wpa-none"))
            ros_connection_message->wifi_security = "WPA/WPA2";
          else if (!strcmp(key_mgmt, "wpa-psk"))
            ros_connection_message->wifi_security = "WPA/WPA2-PSK";
          else if (!strcmp(key_mgmt, "ieee8021x"))
          {
            if (auth_alg && !strcmp(auth_alg, "leap"))
              ros_connection_message->wifi_security = "IEEE 802.1X (LEAP)";
            else
              ros_connection_message->wifi_security = "IEEE 802.1X (Dynamic WEP)";
          }
          else if (!strcmp(key_mgmt, "wpa-eap"))
            ros_connection_message->wifi_security = "WPA/WPA2-EAP";
          else
            ros_connection_message->wifi_security = key_mgmt;
        }
      }
    }
    else if (nm_connection_is_type(connection_data, NM_SETTING_WIRED_SETTING_NAME))
    {
      ros_connection_message->connection_type = oryx_network_manager::NetworkConnection::ETHERNET_CONNECTION;
    }
    else
    {
      ros_connection_message->connection_type = oryx_network_manager::NetworkConnection::OTHER_CONNECTION;
    }

  }
}
static void fill_device_info(NMDevice* device_data, oryx_network_manager::NetworkDevice* ros_device_message)
{
  ros_device_message->iface = nm_device_get_iface(device_data);
  ros_device_message->product = nm_device_get_product(device_data);

  ros_device_message->driver = nm_device_get_driver(device_data);

  if (NM_IS_DEVICE_WIFI (device_data))
  {
    ros_device_message->hardware_address = nm_device_wifi_get_hw_address(NM_DEVICE_WIFI (device_data) );
    ros_device_message->device_type = oryx_network_manager::NetworkDevice::WIFI_DEVICE;
  }
  else if (NM_IS_DEVICE_ETHERNET(device_data))
  {
    ros_device_message->hardware_address = nm_device_ethernet_get_hw_address(NM_DEVICE_ETHERNET (device_data) );
    ros_device_message->device_type = oryx_network_manager::NetworkDevice::ETHERNET_DEVICE;
  }
  else if (NM_IS_DEVICE_MODEM (device_data))
  {
    NMDeviceModemCapabilities caps = nm_device_modem_get_current_capabilities(NM_DEVICE_MODEM(device_data) );
    if (caps & NM_DEVICE_MODEM_CAPABILITY_GSM_UMTS)
      ros_device_message->device_type = oryx_network_manager::NetworkDevice::GSM_DEVICE;
    else if (caps & NM_DEVICE_MODEM_CAPABILITY_CDMA_EVDO)
      ros_device_message->device_type = oryx_network_manager::NetworkDevice::CDMA_DEVICE;
    else
      ros_device_message->device_type = oryx_network_manager::NetworkDevice::MOBILE_BROADBAND_DEVICE;
  }
  else if (NM_IS_DEVICE_WIMAX(device_data))
  {
    ros_device_message->hardware_address = nm_device_wimax_get_hw_address(NM_DEVICE_WIMAX (device_data) );
    ros_device_message->device_type = oryx_network_manager::NetworkDevice::WIMAX_DEVICE;
  }
  else
  {
    ros_device_message->device_type = oryx_network_manager::NetworkDevice::OTHER_DEVICE;
  }

}

//service callback for getting a list of all connections
static bool get_network_connections_callback(oryx_network_manager::GetNetworkConnections::Request& request,
    oryx_network_manager::GetNetworkConnections::Response& response)
{
  NetworkSettings settings(bus);

//get all of the connections and
  GSList* connectionList = settings.get_connections();
  for (; connectionList != NULL; connectionList = connectionList->next)
  {
    NMConnection* connection_data = (NMConnection*) connectionList->data;
    oryx_network_manager::NetworkConnection ros_connection_message;
    fill_connection_info(connection_data, &ros_connection_message);

    response.connections.push_back(ros_connection_message);
  }
  return true;
}
//service callback for getting a list of all active connections
static bool get_active_network_connections_callback(oryx_network_manager::GetActiveNetworkConnections::Request& request,
    oryx_network_manager::GetActiveNetworkConnections::Response& response)
{
  NetworkSettings settings(bus);
  const GPtrArray *connections = nm_client_get_active_connections(client);
  for (unsigned int i = 0; connections && (i < connections->len); i++)
  {
    NMActiveConnection *active_connection = (NMActiveConnection*) g_ptr_array_index (connections, i);
    oryx_network_manager::ActiveNetworkConnection ros_active_connection_message;

    if(nm_active_connection_get_state (active_connection) == NM_ACTIVE_CONNECTION_STATE_ACTIVATED)
    ros_active_connection_message.connection_state = oryx_network_manager::ActiveNetworkConnection::ACTIVATED_STATE;
    else if(nm_active_connection_get_state (active_connection) == NM_ACTIVE_CONNECTION_STATE_ACTIVATING)
    ros_active_connection_message.connection_state = oryx_network_manager::ActiveNetworkConnection::ACTIVATING_STATE;
    else if(nm_active_connection_get_state (active_connection) == NM_ACTIVE_CONNECTION_STATE_DEACTIVATING)
    ros_active_connection_message.connection_state = oryx_network_manager::ActiveNetworkConnection::DEACTIVATING_STATE;
    else if(nm_active_connection_get_state (active_connection) == NM_ACTIVE_CONNECTION_STATE_UNKNOWN)
    ros_active_connection_message.connection_state = oryx_network_manager::ActiveNetworkConnection::UNKNOWN_STATE;

    NMConnection* connection_data = get_connection_by_path(settings, nm_active_connection_get_connection(active_connection));
    fill_connection_info(connection_data, &ros_active_connection_message.connection);

    const GPtrArray *devices = nm_active_connection_get_devices (active_connection);
    if (devices && devices->len > 0)
    {
      NMDevice* device = (NMDevice*)g_ptr_array_index (devices, 0);
      fill_device_info(device, &ros_active_connection_message.device);

      NMIP4Config* ip4_config = nm_device_get_ip4_config(device);
      if(ip4_config){
        const GSList* addresses = nm_ip4_config_get_addresses(ip4_config);
        if (g_slist_length((GSList *) addresses) && addresses->data){
          char ip_address_str[20];
          guint32 ip_address = nm_ip4_address_get_address((NMIP4Address*)addresses->data);
          sprintf(ip_address_str, "%d.%d.%d.%d", (ip_address>>0)&0xFF, (ip_address>>8)&0xFF, (ip_address>>16)&0xFF, (ip_address>>24)&0xFF);
          ros_active_connection_message.ipv4_address = ip_address_str;
        }
      }
    }


    response.active_connections.push_back(ros_active_connection_message);
  }
  return true;
}

//service callback for getting a list of all devices
static bool get_network_devices_callback(oryx_network_manager::GetNetworkDevices::Request& request,
    oryx_network_manager::GetNetworkDevices::Response& response)
{

//get info about devices on the system
  const GPtrArray *devices = nm_client_get_devices(client);
  for (unsigned int i = 0; devices && (i < devices->len); i++)
  {
    NMDevice *data = (NMDevice*) g_ptr_array_index (devices, i);
    oryx_network_manager::NetworkDevice device;
    fill_device_info(data, &device);
    response.devices.push_back(device);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oryx_network_manager_server");
  ros::NodeHandle n;

  ROS_INFO("Starting Oryx Network Manager");
  g_type_init();

  ROS_INFO("Initializing glib main loop");
  loop = g_main_loop_new(NULL, FALSE);

  ROS_INFO("Initializing libnm");
  dbus_threads_init_default();
  bus = dbus_g_bus_get(DBUS_BUS_SYSTEM, NULL);
  client = nm_client_new();

  ROS_INFO("Running glib main loop");
  spawn_g_main_loop(); //spawns main loop in another thread

  ROS_INFO("Loading Network Manager Config");
  set_config(new NetworkManagerConfig("config.txt"));

  ros::ServiceServer get_active_connections_service = n.advertiseService("get_active_network_connections",
      get_active_network_connections_callback);
  ros::ServiceServer get_connections_service = n.advertiseService("get_network_connections",
      get_network_connections_callback);
  ros::ServiceServer get_devices_service = n.advertiseService("get_network_devices", get_network_devices_callback);
  ros::ServiceServer activate_service = n.advertiseService("activate_connection", activate_connection_callback);

  ros::Timer timer = n.createTimer(ros::Duration(UPDATE_DELAY), config_update_callback);

  ros::spin();

  printf("Quitting\n");

//clean up glib main loop
  g_main_loop_quit(loop);
  g_main_loop_unref(loop);

//cleanup libm
  g_object_unref(client);
  dbus_g_connection_unref(bus);

  return 0;
}
