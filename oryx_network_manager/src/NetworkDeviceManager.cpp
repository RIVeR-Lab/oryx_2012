/*
 * NetworkDeviceManager.cpp
 *
 *  Created on: Nov 28, 2012
 *      Author: mitchell
 */

#include "NetworkDeviceManager.h"
#include "MessageUtil.h"

NetworkDeviceManager::NetworkDeviceManager(ros::NodeHandle& handle, NMClient* client) :
    m_handle(handle), m_client(client), msg_seq(0), device_pub(
        handle.advertise<NetworkDevices>("oryx_network_devices", 10, true))
{
  ROS_INFO("Initing Network Device Manager");
  g_signal_connect(m_client, "device-added", G_CALLBACK (device_added_cb), this);
  g_signal_connect(m_client, "device-removed", G_CALLBACK (device_removed_cb), this);

  const GPtrArray *devices = nm_client_get_devices(m_client);
  for (unsigned int i = 0; devices && (i < devices->len); i++)
  {
    NMDevice *device = NM_DEVICE(g_ptr_array_index (devices, i));
    process_added_device(device);
  }
}

void NetworkDeviceManager::process_added_device(NMDevice *device)
{
  ROS_DEBUG("Network Device Added: %s", nm_device_get_iface(device));
  g_signal_connect(device, "state-changed", G_CALLBACK (device_state_changed_cb), this);

  NetworkDevice device_info;
  fill_device_info(device, device_info);
  devices[nm_device_get_iface(device)] = device_info;

  publish_devices();
}
void NetworkDeviceManager::process_removed_device(NMDevice *device)
{
  ROS_DEBUG("Network Device Removed: %s", nm_device_get_iface(device));
  //g_signal_handlers_disconnect_by_func(device, (gpointer)G_CALLBACK (device_state_changed_cb), this);

  devices.erase(nm_device_get_iface(device));

  publish_devices();
}
void NetworkDeviceManager::process_device_state_changed(NMDevice *device, NMDeviceState state)
{
  ROS_DEBUG("Network Device State Changed: %s -> %d", nm_device_get_iface(device), state);

  NetworkDevice device_info;
  fill_device_info(device, device_info);
  devices[nm_device_get_iface(device)] = device_info;

  publish_devices();
}

void NetworkDeviceManager::fill_device_info(NMDevice* device, NetworkDevice& ros_device_message)
{
  ros_device_message.iface = nm_device_get_iface(device);
  ros_device_message.product = nm_device_get_product(device);
  ros_device_message.state = nm_device_get_state(device);

  ros_device_message.driver = nm_device_get_driver(device);

  if (NM_IS_DEVICE_WIFI (device))
  {
    ros_device_message.hardware_address = nm_device_wifi_get_hw_address(NM_DEVICE_WIFI (device) );
    ros_device_message.device_type = oryx_network_manager::NetworkDevice::WIFI_DEVICE;
  }
  else if (NM_IS_DEVICE_ETHERNET(device))
  {
    ros_device_message.hardware_address = nm_device_ethernet_get_hw_address(NM_DEVICE_ETHERNET (device) );
    ros_device_message.device_type = oryx_network_manager::NetworkDevice::ETHERNET_DEVICE;
  }
  else if (NM_IS_DEVICE_MODEM (device))
  {
    NMDeviceModemCapabilities caps = nm_device_modem_get_current_capabilities(NM_DEVICE_MODEM(device) );
    if (caps & NM_DEVICE_MODEM_CAPABILITY_GSM_UMTS)
      ros_device_message.device_type = oryx_network_manager::NetworkDevice::GSM_DEVICE;
    else if (caps & NM_DEVICE_MODEM_CAPABILITY_CDMA_EVDO)
      ros_device_message.device_type = oryx_network_manager::NetworkDevice::CDMA_DEVICE;
    else
      ros_device_message.device_type = oryx_network_manager::NetworkDevice::MOBILE_BROADBAND_DEVICE;
  }
  else if (NM_IS_DEVICE_WIMAX(device))
  {
    ros_device_message.hardware_address = nm_device_wimax_get_hw_address(NM_DEVICE_WIMAX (device) );
    ros_device_message.device_type = oryx_network_manager::NetworkDevice::WIMAX_DEVICE;
  }
  else
  {
    ros_device_message.device_type = oryx_network_manager::NetworkDevice::OTHER_DEVICE;
  }
}

void NetworkDeviceManager::publish_devices()
{
  NetworkDevices devices_message;
  std::map<std::string, NetworkDevice>::iterator itr = devices.begin();
  for (; itr != devices.end(); ++itr)
    devices_message.list.push_back(itr->second);
  fill_header_message(devices_message.header, msg_seq++);
  device_pub.publish(devices_message);
}

void NetworkDeviceManager::device_added_cb(NMClient *client, NMDevice *device, gpointer user_data)
{
  NetworkDeviceManager* manager = (NetworkDeviceManager*) user_data;
  manager->process_added_device(device);
}
void NetworkDeviceManager::device_removed_cb(NMClient *client, NMDevice *device, gpointer user_data)
{
  NetworkDeviceManager* manager = (NetworkDeviceManager*) user_data;
  manager->process_removed_device(device);
}
void NetworkDeviceManager::device_state_changed_cb(NMDevice *device, NMDeviceState state, guint arg2, guint arg3,
    gpointer user_data)
{
  NetworkDeviceManager* manager = (NetworkDeviceManager*) user_data;
  manager->process_device_state_changed(device, state);
}

NetworkDeviceManager::~NetworkDeviceManager()
{
  //g_signal_handlers_disconnect_by_func(m_client, (gpointer)G_CALLBACK (device_added_cb), this);
  //g_signal_handlers_disconnect_by_func(m_client, (gpointer)G_CALLBACK (device_removed_cb), this);
}

