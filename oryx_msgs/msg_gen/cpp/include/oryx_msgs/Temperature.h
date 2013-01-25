/* Auto-generated by genmsg_cpp for file /home/parallels/groovy_workspace/oryx/oryx_msgs/msg/Temperature.msg */
#ifndef ORYX_MSGS_MESSAGE_TEMPERATURE_H
#define ORYX_MSGS_MESSAGE_TEMPERATURE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace oryx_msgs
{
template <class ContainerAllocator>
struct Temperature_ {
  typedef Temperature_<ContainerAllocator> Type;

  Temperature_()
  : temperature_node(0)
  , temperature(0.0)
  , warning_Temp(0.0)
  , danger_Temp(0.0)
  {
  }

  Temperature_(const ContainerAllocator& _alloc)
  : temperature_node(0)
  , temperature(0.0)
  , warning_Temp(0.0)
  , danger_Temp(0.0)
  {
  }

  typedef int32_t _temperature_node_type;
  int32_t temperature_node;

  typedef float _temperature_type;
  float temperature;

  typedef float _warning_Temp_type;
  float warning_Temp;

  typedef float _danger_Temp_type;
  float danger_Temp;


  typedef boost::shared_ptr< ::oryx_msgs::Temperature_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::oryx_msgs::Temperature_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Temperature
typedef  ::oryx_msgs::Temperature_<std::allocator<void> > Temperature;

typedef boost::shared_ptr< ::oryx_msgs::Temperature> TemperaturePtr;
typedef boost::shared_ptr< ::oryx_msgs::Temperature const> TemperatureConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::oryx_msgs::Temperature_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::oryx_msgs::Temperature_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace oryx_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::oryx_msgs::Temperature_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::oryx_msgs::Temperature_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::oryx_msgs::Temperature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e62c9d14f34e94252b4cc03e1d4997da";
  }

  static const char* value(const  ::oryx_msgs::Temperature_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe62c9d14f34e9425ULL;
  static const uint64_t static_value2 = 0x2b4cc03e1d4997daULL;
};

template<class ContainerAllocator>
struct DataType< ::oryx_msgs::Temperature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "oryx_msgs/Temperature";
  }

  static const char* value(const  ::oryx_msgs::Temperature_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::oryx_msgs::Temperature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 temperature_node\n\
float32 temperature\n\
float32 warning_Temp\n\
float32 danger_Temp\n\
\n\
";
  }

  static const char* value(const  ::oryx_msgs::Temperature_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::oryx_msgs::Temperature_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::oryx_msgs::Temperature_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.temperature_node);
    stream.next(m.temperature);
    stream.next(m.warning_Temp);
    stream.next(m.danger_Temp);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Temperature_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::oryx_msgs::Temperature_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::oryx_msgs::Temperature_<ContainerAllocator> & v) 
  {
    s << indent << "temperature_node: ";
    Printer<int32_t>::stream(s, indent + "  ", v.temperature_node);
    s << indent << "temperature: ";
    Printer<float>::stream(s, indent + "  ", v.temperature);
    s << indent << "warning_Temp: ";
    Printer<float>::stream(s, indent + "  ", v.warning_Temp);
    s << indent << "danger_Temp: ";
    Printer<float>::stream(s, indent + "  ", v.danger_Temp);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ORYX_MSGS_MESSAGE_TEMPERATURE_H

