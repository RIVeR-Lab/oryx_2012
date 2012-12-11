/* Auto-generated by genmsg_cpp for file /home/parallels/groovy_workspace/oryx/oryx_msgs/srv/DiagnositicsRegistration.srv */
#ifndef ORYX_MSGS_SERVICE_DIAGNOSITICSREGISTRATION_H
#define ORYX_MSGS_SERVICE_DIAGNOSITICSREGISTRATION_H
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

#include "ros/service_traits.h"

#include "std_msgs/Header.h"



namespace oryx_msgs
{
template <class ContainerAllocator>
struct DiagnositicsRegistrationRequest_ {
  typedef DiagnositicsRegistrationRequest_<ContainerAllocator> Type;

  DiagnositicsRegistrationRequest_()
  : header()
  , node_name()
  , node_type()
  , criticality(0)
  , heartbeat_frequency(0)
  , heartbeat_tolerence(0)
  {
  }

  DiagnositicsRegistrationRequest_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , node_name(_alloc)
  , node_type(_alloc)
  , criticality(0)
  , heartbeat_frequency(0)
  , heartbeat_tolerence(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _node_name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  node_name;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _node_type_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  node_type;

  typedef uint8_t _criticality_type;
  uint8_t criticality;

  typedef uint32_t _heartbeat_frequency_type;
  uint32_t heartbeat_frequency;

  typedef uint32_t _heartbeat_tolerence_type;
  uint32_t heartbeat_tolerence;


  typedef boost::shared_ptr< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct DiagnositicsRegistrationRequest
typedef  ::oryx_msgs::DiagnositicsRegistrationRequest_<std::allocator<void> > DiagnositicsRegistrationRequest;

typedef boost::shared_ptr< ::oryx_msgs::DiagnositicsRegistrationRequest> DiagnositicsRegistrationRequestPtr;
typedef boost::shared_ptr< ::oryx_msgs::DiagnositicsRegistrationRequest const> DiagnositicsRegistrationRequestConstPtr;


template <class ContainerAllocator>
struct DiagnositicsRegistrationResponse_ {
  typedef DiagnositicsRegistrationResponse_<ContainerAllocator> Type;

  DiagnositicsRegistrationResponse_()
  : node_id(0)
  {
  }

  DiagnositicsRegistrationResponse_(const ContainerAllocator& _alloc)
  : node_id(0)
  {
  }

  typedef uint16_t _node_id_type;
  uint16_t node_id;


  typedef boost::shared_ptr< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct DiagnositicsRegistrationResponse
typedef  ::oryx_msgs::DiagnositicsRegistrationResponse_<std::allocator<void> > DiagnositicsRegistrationResponse;

typedef boost::shared_ptr< ::oryx_msgs::DiagnositicsRegistrationResponse> DiagnositicsRegistrationResponsePtr;
typedef boost::shared_ptr< ::oryx_msgs::DiagnositicsRegistrationResponse const> DiagnositicsRegistrationResponseConstPtr;

struct DiagnositicsRegistration
{

typedef DiagnositicsRegistrationRequest Request;
typedef DiagnositicsRegistrationResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct DiagnositicsRegistration
} // namespace oryx_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "934a70629fbc90170de539672a517c1a";
  }

  static const char* value(const  ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x934a70629fbc9017ULL;
  static const uint64_t static_value2 = 0x0de539672a517c1aULL;
};

template<class ContainerAllocator>
struct DataType< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "oryx_msgs/DiagnositicsRegistrationRequest";
  }

  static const char* value(const  ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
\n\
\n\
Header header\n\
\n\
\n\
\n\
\n\
string node_name\n\
\n\
\n\
string node_type\n\
\n\
\n\
\n\
\n\
uint8 criticality\n\
\n\
\n\
\n\
uint32 heartbeat_frequency\n\
\n\
\n\
\n\
\n\
uint32 heartbeat_tolerence\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ee3c79643e54cac2b3e515df2a461d42";
  }

  static const char* value(const  ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xee3c79643e54cac2ULL;
  static const uint64_t static_value2 = 0xb3e515df2a461d42ULL;
};

template<class ContainerAllocator>
struct DataType< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "oryx_msgs/DiagnositicsRegistrationResponse";
  }

  static const char* value(const  ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
uint16 node_id\n\
\n\
\n\
\n\
";
  }

  static const char* value(const  ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.node_name);
    stream.next(m.node_type);
    stream.next(m.criticality);
    stream.next(m.heartbeat_frequency);
    stream.next(m.heartbeat_tolerence);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct DiagnositicsRegistrationRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.node_id);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct DiagnositicsRegistrationResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<oryx_msgs::DiagnositicsRegistration> {
  static const char* value() 
  {
    return "698e2521f84f02743caae3010eb8d359";
  }

  static const char* value(const oryx_msgs::DiagnositicsRegistration&) { return value(); } 
};

template<>
struct DataType<oryx_msgs::DiagnositicsRegistration> {
  static const char* value() 
  {
    return "oryx_msgs/DiagnositicsRegistration";
  }

  static const char* value(const oryx_msgs::DiagnositicsRegistration&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "698e2521f84f02743caae3010eb8d359";
  }

  static const char* value(const oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "oryx_msgs/DiagnositicsRegistration";
  }

  static const char* value(const oryx_msgs::DiagnositicsRegistrationRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "698e2521f84f02743caae3010eb8d359";
  }

  static const char* value(const oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "oryx_msgs/DiagnositicsRegistration";
  }

  static const char* value(const oryx_msgs::DiagnositicsRegistrationResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ORYX_MSGS_SERVICE_DIAGNOSITICSREGISTRATION_H

