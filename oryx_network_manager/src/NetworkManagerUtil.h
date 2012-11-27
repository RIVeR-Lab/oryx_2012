/*
 * NetworkManagerUtil.h
 *
 *  Created on: Nov 24, 2012
 *      Author: mitchell
 */

#ifndef NETWORKMANAGERUTIL_H_
#define NETWORKMANAGERUTIL_H_

#include <stdlib.h>
#include <glib.h>
#include <nm-device.h>
#include <nm-connection.h>
#include <nm-client.h>
#include "NetworkSettings.h"

NMDevice* get_device_by_iface(NMClient *client, const char* iface)
{
  const GPtrArray *devices = nm_client_get_devices(client);
  for (unsigned int i = 0; devices && (i < devices->len); i++)
  {
    NMDevice* device_data = (NMDevice*) g_ptr_array_index (devices, i);
    if(!strcmp(iface, nm_device_get_iface (device_data)))
    return device_data;
  }
  return NULL;
}
NMConnection* get_connection_by_name(NetworkSettings& settings, const char* name)
{
  GSList* connectionList = settings.get_connections();
  for (; connectionList != NULL; connectionList = connectionList->next)
  {
    NMConnection* connection_data = (NMConnection*) connectionList->data;

    NMSettingConnection *connection_setting = nm_connection_get_setting_connection(connection_data);
    if (connection_setting)
    {
      if (!strcmp(name, nm_setting_connection_get_id(connection_setting)))
        return connection_data;
    }
  }
  return NULL;
}
NMConnection* get_connection_by_path(NetworkSettings& settings, const char* path)
{
  GSList* connectionList = settings.get_connections();
  for (; connectionList != NULL; connectionList = connectionList->next)
  {
    NMConnection* connection_data = (NMConnection*) connectionList->data;

    NMSettingConnection *connection_setting = nm_connection_get_setting_connection(connection_data);
    if (connection_setting)
    {
      if (!strcmp(nm_connection_get_path(connection_data), path))
        return connection_data;
    }
  }
  return NULL;
}

bool activate_connection(NMClient* client, NetworkSettings& settings, std::string iface, std::string connection_name)
{
  NMDevice * device = get_device_by_iface(client, iface.c_str());
  if (device == NULL)
  {
    return false;
  }

  NMConnection * connection = get_connection_by_name(settings, connection_name.c_str());
  if (connection == NULL)
  {
    return false;
  }

  ROS_INFO("Activiting Connection: '%s' on %s", connection_name.c_str(), iface.c_str());

  nm_client_activate_connection(client, connection, device, NULL, NULL, NULL);

  return true;
}

#endif /* NETWORKMANAGERUTIL_H_ */
