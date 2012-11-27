/*
 * NetworkSettings.h
 *
 *  Created on: Nov 18, 2012
 *      Author: mitchell
 */

#ifndef NETWORKSETTINGS_H_
#define NETWORKSETTINGS_H_

#include <nm-remote-settings.h>

class NetworkSettings
{
public:
  NetworkSettings(DBusGConnection *bus)
  {
    connections = NULL;
    settings = nm_remote_settings_new(bus); //TODO add error checking
    g_signal_connect(settings, NM_REMOTE_SETTINGS_CONNECTIONS_READ, G_CALLBACK(got_connections_callback), this);

    int start_time = time(NULL);

    while(connections==NULL && time(NULL)-start_time<2)
      usleep(1000);
    if(time(NULL)-start_time>=2)
      ROS_WARN("Timed out while getting Network Settings...");
  }
  ~NetworkSettings()
  {
    g_slist_free(connections);
    g_object_unref(settings);
  }
  GSList * get_connections(){
    return connections;
  }
  static void got_connections_callback(NMRemoteSettings *settings, gpointer user_data)
  {
    ((NetworkSettings*)user_data)->connections = nm_remote_settings_list_connections(settings);
  }
private:
  GSList *connections;
  NMRemoteSettings *settings;
};

#endif /* NETWORKSETTINGS_H_ */
