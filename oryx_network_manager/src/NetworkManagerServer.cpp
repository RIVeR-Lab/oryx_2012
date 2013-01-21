#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include <glib.h>
#include <dbus/dbus-glib.h>
#include <dbus/dbus.h>
#include <pthread.h>

#include "NetworkDeviceManager.h"
#include "NetworkConnectionManager.h"
#include "ActiveNetworkConnectionManager.h"
#include "ActivateConnectionService.h"
#include "NetworkConfigurationManager.h"

/*
 * The main entry point for the network manager server
 * The server allows for automatic and remote configuration of a computer's network connections
 */


pthread_t g_main_loop_thread;
//spawned in new thread to run the glib main loop, this loop handles dispatching events
static void* run_main_loop(void* loop)
{
  g_main_loop_run ((GMainLoop*)loop);
  pthread_exit(NULL);
}

//spawns the glib main loop in a new thread
static void spawn_g_main_loop(GMainLoop* loop)
{
  int rc = pthread_create(&g_main_loop_thread, NULL, run_main_loop, loop);
  if (rc)
  {
    ROS_ERROR("ERROR; return code from pthread_create() is %d\n", rc);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oryx_network_manager_server");
  ros::NodeHandle n;
  ros::NodeHandle private_n("~");

  ROS_INFO("Starting Oryx Network Manager");
  g_type_init();

  ROS_INFO("Initializing glib main loop");
  GMainLoop* loop = g_main_loop_new(NULL, FALSE);

  ROS_INFO("Initializing libnm");
  dbus_threads_init_default();
  DBusGConnection* bus = dbus_g_bus_get(DBUS_BUS_SYSTEM, NULL);
  NMClient* client = nm_client_new();
  NMRemoteSettings* remote_settings = nm_remote_settings_new(bus);

  ROS_INFO("Running glib main loop");
  spawn_g_main_loop(loop); //spawns main loop in another thread

  NetworkDeviceManager deviceManager(n, client);
  NetworkConnectionManager connectionManager(n, remote_settings);
  ActiveNetworkConnectionManager activeConnectionManager(n, client);
  ActivateConnectionService activateConnectionService(n, client, remote_settings);
  NetworkConfigurationManager configurationManager(n, client, remote_settings);


  ROS_INFO("Waiting 2 seconds to retrieve connection info");
  ros::Time time = ros::Time::now() + ros::Duration(2);
  while(ros::ok() && time>ros::Time::now()){
    ros::spinOnce();
  }

  std::string config_file_name;
  if(private_n.getParam("config_file", config_file_name)){
    configurationManager.load_config_from_file(config_file_name.c_str());
  }
  else
    ROS_INFO("No config file specified");

  ros::spin();

  printf("Quitting\n");

//clean up glib main loop
  g_main_loop_quit(loop);
  g_main_loop_unref(loop);

  printf("Main Loop Stopped\n");

//cleanup libm
  g_object_unref(client);
  dbus_g_connection_unref(bus);


  printf("Waiting on other threads to exit\n");
  //wait for main loop thread to terminate
  pthread_exit(NULL);

  return 0;
}
