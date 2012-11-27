/*
 * NetworkManagerConfig.h
 *
 *  Created on: Nov 24, 2012
 *      Author: mitchell
 */

#ifndef NETWORKMANAGERCONFIG_H_
#define NETWORKMANAGERCONFIG_H_

#include <string>
#include <map>
#include "NetworkManagerUtil.h"

class NetworkManagerConfig
{
private:
  std::string name;
  std::map<std::string, std::string> config_connections;
public:
  NetworkManagerConfig(const char* config_file)
  {
    FILE * pFile = fopen(config_file, "r");
    if (!pFile)
    {
      ROS_WARN("Error reading config file: %s", config_file);
      return;
    }
    char name_tmp[100];
    int num_read;
    num_read = fscanf(pFile, "%[^\n\r]", name_tmp);
    if (num_read != 1)
    {
      ROS_WARN("Badly formatted config file: %s", config_file);
    }
    else
    {
      name = name_tmp;

      char iface[100];
      char connection_name[100];
      do
      {
        num_read = fscanf(pFile, "%*[ \n\t]%[^,],%[^\n\r]", iface, connection_name);
        if (num_read == 2)
          config_connections[iface] = connection_name;
      } while (num_read == 2);
      if (num_read > 0)
      {
        ROS_WARN("Badly formatted config file: %s. only read %i items on connection line", config_file, num_read);
      }
    }

    fclose(pFile);
  }
  ;
  std::string getName(){
    return name;
  };
  void update(DBusGConnection *bus, NMClient *client)
  {
    NetworkSettings settings(bus);
    const GPtrArray *active_connections = nm_client_get_active_connections(client);
    std::map<std::string, std::string>::iterator itr;
    for (itr = config_connections.begin(); itr != config_connections.end(); ++itr)
    {
      std::string iface = itr->first;
      std::string connection_name = itr->second;
      bool found = false;

      for (unsigned int i = 0; active_connections && (i < active_connections->len); i++)
      {
        NMActiveConnection *active_connection = (NMActiveConnection*) g_ptr_array_index(active_connections, i);

        NMConnection* connection_data = get_connection_by_path(settings,
            nm_active_connection_get_connection(active_connection));
        NMSettingConnection *connection_setting = nm_connection_get_setting_connection(connection_data);
        if (connection_setting)
        {
          if (connection_name == nm_setting_connection_get_id(connection_setting))
          {
            const GPtrArray *devices = nm_active_connection_get_devices(active_connection);
            if (devices && devices->len > 0)
            {
              NMDevice* device_data = (NMDevice*) g_ptr_array_index(devices, 0);
              if(iface==nm_device_get_iface(device_data))
                found = true;
            }
          }
        }
      }
      if(!found){
        ROS_INFO("Not connected to %s on %s. Attempting to connect now", connection_name.c_str(), iface.c_str());
        if(!activate_connection(client, settings, iface, connection_name))
          ROS_WARN("Failed to connect to %s on %s", connection_name.c_str(), iface.c_str());
      }
    }
  }
  ;
};

#endif /* NETWORKMANAGERCONFIG_H_ */
