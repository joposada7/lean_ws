// Generated by gencpp from file acl_msgs/QuadAttCmd.msg
// DO NOT EDIT!


#ifndef ACL_MSGS_MESSAGE_QUADATTCMD_H
#define ACL_MSGS_MESSAGE_QUADATTCMD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace acl_msgs
{
template <class ContainerAllocator>
struct QuadAttCmd_
{
  typedef QuadAttCmd_<ContainerAllocator> Type;

  QuadAttCmd_()
    : header()
    , att()
    , rate()
    , throttle(0.0)
    , att_status(0)  {
    }
  QuadAttCmd_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , att(_alloc)
    , rate(_alloc)
    , throttle(0.0)
    , att_status(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _att_type;
  _att_type att;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _rate_type;
  _rate_type rate;

   typedef double _throttle_type;
  _throttle_type throttle;

   typedef int16_t _att_status_type;
  _att_status_type att_status;





  typedef boost::shared_ptr< ::acl_msgs::QuadAttCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::acl_msgs::QuadAttCmd_<ContainerAllocator> const> ConstPtr;

}; // struct QuadAttCmd_

typedef ::acl_msgs::QuadAttCmd_<std::allocator<void> > QuadAttCmd;

typedef boost::shared_ptr< ::acl_msgs::QuadAttCmd > QuadAttCmdPtr;
typedef boost::shared_ptr< ::acl_msgs::QuadAttCmd const> QuadAttCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::acl_msgs::QuadAttCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::acl_msgs::QuadAttCmd_<ContainerAllocator1> & lhs, const ::acl_msgs::QuadAttCmd_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.att == rhs.att &&
    lhs.rate == rhs.rate &&
    lhs.throttle == rhs.throttle &&
    lhs.att_status == rhs.att_status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::acl_msgs::QuadAttCmd_<ContainerAllocator1> & lhs, const ::acl_msgs::QuadAttCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace acl_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::acl_msgs::QuadAttCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::acl_msgs::QuadAttCmd_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::acl_msgs::QuadAttCmd_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5849d441faf7e1cf53891f700914d00f";
  }

  static const char* value(const ::acl_msgs::QuadAttCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5849d441faf7e1cfULL;
  static const uint64_t static_value2 = 0x53891f700914d00fULL;
};

template<class ContainerAllocator>
struct DataType< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "acl_msgs/QuadAttCmd";
  }

  static const char* value(const ::acl_msgs::QuadAttCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"geometry_msgs/Quaternion att\n"
"geometry_msgs/Vector3 rate\n"
"float64 throttle\n"
"int16 att_status\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::acl_msgs::QuadAttCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.att);
      stream.next(m.rate);
      stream.next(m.throttle);
      stream.next(m.att_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QuadAttCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::acl_msgs::QuadAttCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::acl_msgs::QuadAttCmd_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "att: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.att);
    s << indent << "rate: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.rate);
    s << indent << "throttle: ";
    Printer<double>::stream(s, indent + "  ", v.throttle);
    s << indent << "att_status: ";
    Printer<int16_t>::stream(s, indent + "  ", v.att_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ACL_MSGS_MESSAGE_QUADATTCMD_H
