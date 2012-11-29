/*
 * NetworkManagerCLI.cpp
 *
 *  Created on: Nov 18, 2012
 *      Author: mitchell
 */

#include "ros/ros.h"
#include "oryx_network_manager/NetworkConnections.h"
#include "oryx_network_manager/NetworkDevices.h"
#include "oryx_network_manager/ActiveNetworkConnections.h"
#include "oryx_network_manager/ActivateConnection.h"
#include <sstream>

using namespace oryx_network_manager;

void list_connections_callback(const NetworkConnections::ConstPtr& connections, bool run_once)
{
  printf("Oryx Network Connections\n");
  printf("------------------------------------------------\n");
  for (unsigned int i = 0; i < connections->list.size(); ++i)
  {
    NetworkConnection connection = connections->list[i];
    printf("\t%s (%s)\n", connection.name.c_str(), connection.path.c_str());

    printf("\t\t%s Connection\n", connection.connection_type.c_str());

    if (!connection.wifi_ssid.empty())
      printf("\t\tSSID: %s\n", connection.wifi_ssid.c_str());
    if (!connection.wifi_security.empty())
      printf("\t\tSecurity: %s\n", connection.wifi_security.c_str());
    printf("\n");
  }
  if (run_once)
    ros::shutdown();
}

void list_connections(bool run_once)
{
  ros::NodeHandle n;
  ros::Subscriber connections_sub = n.subscribe<NetworkConnections>("oryx_network_connections", 10,
      boost::bind(list_connections_callback, _1, run_once));
  ros::spin();
}

void list_active_connections_callback(const ActiveNetworkConnections::ConstPtr& active_connections, bool run_once)
{
  printf("Oryx Active Network Connections\n");
  printf("------------------------------------------------\n");
  for (unsigned int i = 0; i < active_connections->list.size(); ++i)
  {
    ActiveNetworkConnection connection = active_connections->list[i];
    printf("\t%s on %s: %d\n", connection.connection.name.c_str(), connection.device.iface.c_str(),
        connection.connection_state);
    if (!connection.ipv4_address.empty())
      printf("\t\tIPv4 Address: %s\n", connection.ipv4_address.c_str());
    printf("\n");
  }
  if(run_once)
    ros::shutdown();
}
void list_active_connections(bool run_once)
{
  ros::NodeHandle n;
  ros::Subscriber connections_sub = n.subscribe<ActiveNetworkConnections>("oryx_active_network_connections", 10,
      boost::bind(list_active_connections_callback, _1, run_once));
  ros::spin();
}

void list_devices_callback(const NetworkDevices::ConstPtr& devices, bool run_once)
{
  printf("Oryx Network Devices\n");
  printf("------------------------------------------------\n");
  for (unsigned int i = 0; i < devices->list.size(); ++i)
  {
    oryx_network_manager::NetworkDevice device = devices->list[i];
    printf("\t%s (%s)\n", device.iface.c_str(), device.product.c_str());
    printf("\t\t%s Device\n", device.device_type.c_str());
    if (!device.driver.empty())
      printf("\t\tDriver: %s\n", device.driver.c_str());
    if (!device.hardware_address.empty())
      printf("\t\tHardware Address: %s\n", device.hardware_address.c_str());
    printf("\n");
  }
  if(run_once)
    ros::shutdown();
}

void list_devices(bool run_once)
{
  ros::NodeHandle n;
  ros::Subscriber connections_sub = n.subscribe<NetworkDevices>("oryx_network_devices", 10,
      boost::bind(list_devices_callback, _1, run_once));
  ros::spin();
}

void connect(const char* iface, const char* connection_name)
{
  printf("Connecting to '%s' on '%s'\n", connection_name, iface);
  oryx_network_manager::ActivateConnection activateConnection;
  activateConnection.request.connection_name = connection_name;
  activateConnection.request.device_iface = iface;
  if (ros::service::call("oryx_activate_network_connection", activateConnection))
  {
    if (!activateConnection.response.message.empty())
      printf("%s\n", activateConnection.response.message.c_str());
  }
  else
    printf("Service call to network manager failed. Make sure that the server is running and you are connected.\n");
}

void help(const char* message)
{
  if (strcmp("", message) != 0)
    printf("%s\n", message);
  printf("Oryx Network Manager CLI Help:\n");
  printf("The following commands are supported\n");
  printf("\t%s\t\t\t\t\t%s\n", "cli help", "Show this message");
  printf("\t%s\t%s\n", "cli list [devices|connections|active]", "list the connections and devices on oryx");
  printf("\t%s\t\t%s\n", "cli connect <iface> <connection_name>",
      "connect to a specified connection on a specific device");
  printf("\t%s\t%s\n", "cli mon [devices|connections|active]",
      "listen for updates to the  connections and devices on oryx and display them");
}

int main(int argc, char **argv) //first arg is program name
{
  ros::init(argc, argv, "oryx_network_manager_cli", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  if (argc <= 1)
    help("");
  else if (strcmp("help", argv[1]) == 0)
    help("");
  else if (strcmp("list", argv[1]) == 0)
  {
    if (argc == 2)
      list_connections(true);
    else if (argc == 3)
    {
      if (strcmp("connections", argv[2]) == 0)
        list_connections(true);
      else if (strcmp("devices", argv[2]) == 0)
        list_devices(true);
      else if (strcmp("active", argv[2]) == 0)
        list_active_connections(true);
      else
        help("Unsupported list type\n");
    }
    else
      help("list only takes one argument");
  }
  else if (strcmp("mon", argv[1]) == 0)
  {
    if (argc == 2)
      list_connections(false);
    else if (argc == 3)
    {
      if (strcmp("connections", argv[2]) == 0)
        list_connections(false);
      else if (strcmp("devices", argv[2]) == 0)
        list_devices(false);
      else if (strcmp("active", argv[2]) == 0)
        list_active_connections(false);
      else
        help("Unsupported mon type\n");
    }
    else
      help("mon only takes one argument");
  }
  else if (strcmp("connect", argv[1]) == 0)
  {
    if (argc == 4)
      connect(argv[2], argv[3]);
    else
      help("connect takes 2 arguments");
  }
  else
  {
    char tmp[200];
    strcpy(tmp, "Unknown command: '");
    strcat(tmp, argv[1]);
    strcat(tmp, "'");
    help(tmp);
  }

  return 0;
}

