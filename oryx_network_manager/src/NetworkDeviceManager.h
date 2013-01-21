/*
 * NetworkDeviceManager.h
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 *
 * A class which handles changes in the connected network devices and provides a service to retrieve the connected devices
 */

#ifndef NETWORKDEVICEMANAGER_H_
#define NETWORKDEVICEMANAGER_H_

#include "ros/ros.h"
#include "oryx_network_manager/NetworkDevices.h"
#include <nm-client.h>
#include <nm-device.h>
#include <nm-device-wifi.h>
#include <nm-device-ethernet.h>
#include <nm-device-modem.h>
#include <nm-device-wimax.h>
#include <map>

using namespace oryx_network_manager;

class NetworkDeviceManager
{
public:
  NetworkDeviceManager(ros::NodeHandle& handle, NMClient* client);
  virtual ~NetworkDeviceManager();
private:
  void process_added_device(NMDevice *device);
  void process_removed_device(NMDevice *device);
  void process_device_state_changed(NMDevice *device, NMDeviceState state);
  void fill_device_info(NMDevice* device, NetworkDevice& ros_device_message);

  static void device_added_cb(NMClient *client, NMDevice *device, gpointer user_data);
  static void device_removed_cb(NMClient *client, NMDevice *device, gpointer user_data);
  static void device_state_changed_cb(NMDevice *device, NMDeviceState state, guint arg2, guint arg3,
      gpointer user_data);
  void publish_devices();
private:
  ros::NodeHandle& m_handle;
  NMClient* m_client;
  uint32_t msg_seq;
  ros::Publisher device_pub;
  std::map<std::string, NetworkDevice> devices;
};

#endif /* NETWORKDEVICEMANAGER_H_ */
