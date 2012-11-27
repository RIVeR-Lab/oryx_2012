/*
 * NetworkManagerCLI.cpp
 *
 *  Created on: Nov 18, 2012
 *      Author: mitchell
 */

#include "ros/ros.h"
#include "oryx_network_manager/GetNetworkConnections.h"
#include "oryx_network_manager/GetNetworkDevices.h"
#include "oryx_network_manager/GetActiveNetworkConnections.h"
#include "oryx_network_manager/ActivateConnection.h"
#include <sstream>

void list_connections()
{
  oryx_network_manager::GetNetworkConnections networkConnections;
  if (ros::service::call("get_network_connections", networkConnections))
  {
    printf("Oryx Network Connections\n");
    printf("------------------------------------------------\n");
    for (unsigned int i = 0; i < networkConnections.response.connections.size(); ++i)
    {
      oryx_network_manager::NetworkConnection connection = networkConnections.response.connections[i];
      printf("\t%s (%s)\n", connection.name.c_str(), connection.uuid.c_str());

      printf("\t\t%s Connection\n", connection.connection_type.c_str());

      if(!connection.wifi_ssid.empty())
        printf("\t\tSSID: %s\n", connection.wifi_ssid.c_str());
      if(!connection.wifi_security.empty())
        printf("\t\tSecurity: %s\n", connection.wifi_security.c_str());
      printf("\n");
    }
  }
  else
    printf("Service call to network manager failed. Make sure that the server is running and you are connected.\n");
}
void list_active_connections()
{
  oryx_network_manager::GetActiveNetworkConnections networkConnections;
  if (ros::service::call("get_active_network_connections", networkConnections))
  {
    printf("Oryx Active Network Connections\n");
    printf("------------------------------------------------\n");
    for (unsigned int i = 0; i < networkConnections.response.active_connections.size(); ++i)
    {
      oryx_network_manager::ActiveNetworkConnection connection = networkConnections.response.active_connections[i];
      printf("\t%s on %s: %d\n", connection.connection.name.c_str(), connection.device.iface.c_str(), connection.connection_state);
      if(!connection.ipv4_address.empty())
        printf("\t\tIPv4 Address: %s\n", connection.ipv4_address.c_str());
      printf("\n");
    }
  }
  else
    printf("Service call to network manager failed. Make sure that the server is running and you are connected.\n");
}

void list_devices()
{
  oryx_network_manager::GetNetworkDevices networkDevices;
  if (ros::service::call("get_network_devices", networkDevices))
  {
    printf("Oryx Network Devices\n");
    printf("------------------------------------------------\n");
    for (unsigned int i = 0; i < networkDevices.response.devices.size(); ++i)
    {
      oryx_network_manager::NetworkDevice device = networkDevices.response.devices[i];
      printf("\t%s (%s)\n", device.iface.c_str(), device.product.c_str());
      printf("\t\t%s Device\n", device.device_type.c_str());
      if(!device.driver.empty())
        printf("\t\tDriver: %s\n", device.driver.c_str());
      if(!device.hardware_address.empty())
        printf("\t\tHardware Address: %s\n", device.hardware_address.c_str());
      printf("\n");
    }
  }
  else
    printf("Service call to network manager failed. Make sure that the server is running and you are connected.\n");
}

void connect(const char* iface, const char* connection_name)
{
  printf("Connecting to '%s' on '%s'\n", connection_name, iface);
  oryx_network_manager::ActivateConnection activateConnection;
  activateConnection.request.connection_name = connection_name;
  activateConnection.request.device_iface = iface;
  if (ros::service::call("activate_connection", activateConnection))
  {
    if(!activateConnection.response.message.empty())
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
  printf("\t%s\t%s\n", "cli list [devices] [connections] [active]", "list all the connections and devices on oryx");
  printf("\t%s\t\t%s\n", "cli connect <iface> <connection_name>",
      "connect to a specified connection on a specific device");
}

int main(int argc, char **argv) //first arg is program name
{
  ros::init(argc, argv, "oryx_network_manager_cli");
  ros::NodeHandle n;

  if (argc <= 1)
    help("");
  else if (strcmp("help", argv[1]) == 0)
    help("");
  else if (strcmp("list", argv[1]) == 0)
  {
    if (argc == 2)
      list_connections();
    else
    {
      for (int i = 2; i < argc; ++i)
      {
        char* arg = argv[i];
        if (strcmp("connections", arg) == 0)
          list_connections();
        else if (strcmp("devices", arg) == 0)
          list_devices();
        else if (strcmp("active", arg)==0)
          list_active_connections();
        else
          printf("Unsupported list type: %s\n", arg);
      }
    }
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
    char tmp[100];
    strcpy(tmp, "Unknown command: '");
    strcat(tmp, argv[1]);
    strcat(tmp, "'");
    help(tmp);
  }

  return 0;
}

