/* Auto-generated by genmsg_cpp for file /home/parallels/groovy_workspace/oryx/epos_manager/msg/EPOSControl.msg */
#ifndef EPOS_MANAGER_MESSAGE_EPOSCONTROL_H
#define EPOS_MANAGER_MESSAGE_EPOSCONTROL_H
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


namespace epos_manager
{
template <class ContainerAllocator>
struct EPOSControl_ {
  typedef EPOSControl_<ContainerAllocator> Type;

  EPOSControl_()
  : node_id(0)
  , control_mode(0)
  , setpoint(0)
  {
  }

  EPOSControl_(const ContainerAllocator& _alloc)
  : node_id(0)
  , control_mode(0)
  , setpoint(0)
  {
  }

  typedef uint16_t _node_id_type;
  uint16_t node_id;

  typedef uint8_t _control_mode_type;
  uint8_t control_mode;

  typedef int32_t _setpoint_type;
  int32_t setpoint;

  enum { VELOCITY = 1 };
  enum { ABSOLUTE_POSITION = 2 };
  enum { ABSOLUTE_POSITION_IMMEDIATE = 3 };
  enum { RELATIVE_POSITION = 4 };
  enum { RELATIVE_POSITION_IMMEDIATE = 5 };

  typedef boost::shared_ptr< ::epos_manager::EPOSControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::epos_manager::EPOSControl_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct EPOSControl
typedef  ::epos_manager::EPOSControl_<std::allocator<void> > EPOSControl;

typedef boost::shared_ptr< ::epos_manager::EPOSControl> EPOSControlPtr;
typedef boost::shared_ptr< ::epos_manager::EPOSControl const> EPOSControlConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::epos_manager::EPOSControl_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::epos_manager::EPOSControl_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace epos_manager

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::epos_manager::EPOSControl_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::epos_manager::EPOSControl_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::epos_manager::EPOSControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "a188b630eb63b57363acb91954210f81";
  }

  static const char* value(const  ::epos_manager::EPOSControl_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xa188b630eb63b573ULL;
  static const uint64_t static_value2 = 0x63acb91954210f81ULL;
};

template<class ContainerAllocator>
struct DataType< ::epos_manager::EPOSControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "epos_manager/EPOSControl";
  }

  static const char* value(const  ::epos_manager::EPOSControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::epos_manager::EPOSControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# This represents a control structure for a Maxon motor attached to an EPOS 2 controller.  \n\
# The node_id is the node_id set on the epos controller itself\n\
# The control_mode corresponds with the desired mode of control\n\
# The setpoint corresponds with the desired value for the selected mode\n\
\n\
# Control Mode Options\n\
uint8 VELOCITY =1\n\
uint8 ABSOLUTE_POSITION = 2\n\
uint8 ABSOLUTE_POSITION_IMMEDIATE = 3\n\
uint8 RELATIVE_POSITION = 4\n\
uint8 RELATIVE_POSITION_IMMEDIATE = 5\n\
\n\
\n\
uint16 node_id\n\
uint8 control_mode\n\
int32 setpoint\n\
\n\
\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::epos_manager::EPOSControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::epos_manager::EPOSControl_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::epos_manager::EPOSControl_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.node_id);
    stream.next(m.control_mode);
    stream.next(m.setpoint);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EPOSControl_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::epos_manager::EPOSControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::epos_manager::EPOSControl_<ContainerAllocator> & v) 
  {
    s << indent << "node_id: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.node_id);
    s << indent << "control_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.control_mode);
    s << indent << "setpoint: ";
    Printer<int32_t>::stream(s, indent + "  ", v.setpoint);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EPOS_MANAGER_MESSAGE_EPOSCONTROL_H
