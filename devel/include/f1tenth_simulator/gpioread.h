// Generated by gencpp from file f1tenth_simulator/gpioread.msg
// DO NOT EDIT!


#ifndef F1TENTH_SIMULATOR_MESSAGE_GPIOREAD_H
#define F1TENTH_SIMULATOR_MESSAGE_GPIOREAD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace f1tenth_simulator
{
template <class ContainerAllocator>
struct gpioread_
{
  typedef gpioread_<ContainerAllocator> Type;

  gpioread_()
    : header()
    , state(false)
    , pin()  {
    }
  gpioread_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , state(false)
    , pin(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _state_type;
  _state_type state;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _pin_type;
  _pin_type pin;





  typedef boost::shared_ptr< ::f1tenth_simulator::gpioread_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::f1tenth_simulator::gpioread_<ContainerAllocator> const> ConstPtr;

}; // struct gpioread_

typedef ::f1tenth_simulator::gpioread_<std::allocator<void> > gpioread;

typedef boost::shared_ptr< ::f1tenth_simulator::gpioread > gpioreadPtr;
typedef boost::shared_ptr< ::f1tenth_simulator::gpioread const> gpioreadConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::f1tenth_simulator::gpioread_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::f1tenth_simulator::gpioread_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::f1tenth_simulator::gpioread_<ContainerAllocator1> & lhs, const ::f1tenth_simulator::gpioread_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.state == rhs.state &&
    lhs.pin == rhs.pin;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::f1tenth_simulator::gpioread_<ContainerAllocator1> & lhs, const ::f1tenth_simulator::gpioread_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace f1tenth_simulator

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::f1tenth_simulator::gpioread_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::f1tenth_simulator::gpioread_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::f1tenth_simulator::gpioread_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::f1tenth_simulator::gpioread_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::f1tenth_simulator::gpioread_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::f1tenth_simulator::gpioread_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::f1tenth_simulator::gpioread_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a6977a6ae8bef9739a477a4e7511d644";
  }

  static const char* value(const ::f1tenth_simulator::gpioread_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa6977a6ae8bef973ULL;
  static const uint64_t static_value2 = 0x9a477a4e7511d644ULL;
};

template<class ContainerAllocator>
struct DataType< ::f1tenth_simulator::gpioread_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f1tenth_simulator/gpioread";
  }

  static const char* value(const ::f1tenth_simulator::gpioread_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::f1tenth_simulator::gpioread_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"# the state of the pin. true means high, false means low.\n"
"bool state\n"
"\n"
"# the pin that has changed. Represented as a string because\n"
"# some pins are named things like '12', while others are named\n"
"# things like 'P8' depending on your hardware.\n"
"string pin\n"
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
;
  }

  static const char* value(const ::f1tenth_simulator::gpioread_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::f1tenth_simulator::gpioread_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.state);
      stream.next(m.pin);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gpioread_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::f1tenth_simulator::gpioread_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::f1tenth_simulator::gpioread_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
    s << indent << "pin: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.pin);
  }
};

} // namespace message_operations
} // namespace ros

#endif // F1TENTH_SIMULATOR_MESSAGE_GPIOREAD_H
