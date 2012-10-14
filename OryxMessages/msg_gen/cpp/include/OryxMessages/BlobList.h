/* Auto-generated by genmsg_cpp for file /home/oryx/ros_workspace/Oryx/OryxMessages/msg/BlobList.msg */
#ifndef ORYXMESSAGES_MESSAGE_BLOBLIST_H
#define ORYXMESSAGES_MESSAGE_BLOBLIST_H
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

#include "OryxMessages/Blob.h"

namespace OryxMessages
{
template <class ContainerAllocator>
struct BlobList_ {
  typedef BlobList_<ContainerAllocator> Type;

  BlobList_()
  : blobs()
  , blobCount(0)
  , color(0)
  {
  }

  BlobList_(const ContainerAllocator& _alloc)
  : blobs(_alloc)
  , blobCount(0)
  , color(0)
  {
  }

  typedef std::vector< ::OryxMessages::Blob_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::OryxMessages::Blob_<ContainerAllocator> >::other >  _blobs_type;
  std::vector< ::OryxMessages::Blob_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::OryxMessages::Blob_<ContainerAllocator> >::other >  blobs;

  typedef int32_t _blobCount_type;
  int32_t blobCount;

  typedef int8_t _color_type;
  int8_t color;


  ROS_DEPRECATED uint32_t get_blobs_size() const { return (uint32_t)blobs.size(); }
  ROS_DEPRECATED void set_blobs_size(uint32_t size) { blobs.resize((size_t)size); }
  ROS_DEPRECATED void get_blobs_vec(std::vector< ::OryxMessages::Blob_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::OryxMessages::Blob_<ContainerAllocator> >::other > & vec) const { vec = this->blobs; }
  ROS_DEPRECATED void set_blobs_vec(const std::vector< ::OryxMessages::Blob_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::OryxMessages::Blob_<ContainerAllocator> >::other > & vec) { this->blobs = vec; }
private:
  static const char* __s_getDataType_() { return "OryxMessages/BlobList"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "76c461d0b54f222fd763ee65e508f7e8"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Blob[] blobs\n\
int32 blobCount\n\
int8 color\n\
\n\
================================================================================\n\
MSG: OryxMessages/Blob\n\
int32 x\n\
int32 y\n\
int32 size\n\
int32 radius\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, blobs);
    ros::serialization::serialize(stream, blobCount);
    ros::serialization::serialize(stream, color);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, blobs);
    ros::serialization::deserialize(stream, blobCount);
    ros::serialization::deserialize(stream, color);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(blobs);
    size += ros::serialization::serializationLength(blobCount);
    size += ros::serialization::serializationLength(color);
    return size;
  }

  typedef boost::shared_ptr< ::OryxMessages::BlobList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::OryxMessages::BlobList_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BlobList
typedef  ::OryxMessages::BlobList_<std::allocator<void> > BlobList;

typedef boost::shared_ptr< ::OryxMessages::BlobList> BlobListPtr;
typedef boost::shared_ptr< ::OryxMessages::BlobList const> BlobListConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::OryxMessages::BlobList_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::OryxMessages::BlobList_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace OryxMessages

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::OryxMessages::BlobList_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::OryxMessages::BlobList_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::OryxMessages::BlobList_<ContainerAllocator> > {
  static const char* value() 
  {
    return "76c461d0b54f222fd763ee65e508f7e8";
  }

  static const char* value(const  ::OryxMessages::BlobList_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x76c461d0b54f222fULL;
  static const uint64_t static_value2 = 0xd763ee65e508f7e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::OryxMessages::BlobList_<ContainerAllocator> > {
  static const char* value() 
  {
    return "OryxMessages/BlobList";
  }

  static const char* value(const  ::OryxMessages::BlobList_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::OryxMessages::BlobList_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Blob[] blobs\n\
int32 blobCount\n\
int8 color\n\
\n\
================================================================================\n\
MSG: OryxMessages/Blob\n\
int32 x\n\
int32 y\n\
int32 size\n\
int32 radius\n\
\n\
";
  }

  static const char* value(const  ::OryxMessages::BlobList_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::OryxMessages::BlobList_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.blobs);
    stream.next(m.blobCount);
    stream.next(m.color);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BlobList_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::OryxMessages::BlobList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::OryxMessages::BlobList_<ContainerAllocator> & v) 
  {
    s << indent << "blobs[]" << std::endl;
    for (size_t i = 0; i < v.blobs.size(); ++i)
    {
      s << indent << "  blobs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::OryxMessages::Blob_<ContainerAllocator> >::stream(s, indent + "    ", v.blobs[i]);
    }
    s << indent << "blobCount: ";
    Printer<int32_t>::stream(s, indent + "  ", v.blobCount);
    s << indent << "color: ";
    Printer<int8_t>::stream(s, indent + "  ", v.color);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ORYXMESSAGES_MESSAGE_BLOBLIST_H
