// Generated by gencpp from file rosserial_msgs/Log.msg
// DO NOT EDIT!


#ifndef ROSSERIAL_MSGS_MESSAGE_LOG_H
#define ROSSERIAL_MSGS_MESSAGE_LOG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosserial_msgs
{
template <class ContainerAllocator>
struct Log_
{
  typedef Log_<ContainerAllocator> Type;

  Log_()
    : level(0)
    , msg()  {
    }
  Log_(const ContainerAllocator& _alloc)
    : level(0)
    , msg(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _level_type;
  _level_type level;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  _msg_type msg;



  enum {
    ROSDEBUG = 0u,
    INFO = 1u,
    WARN = 2u,
    ERROR = 3u,
    FATAL = 4u,
  };


  typedef boost::shared_ptr< ::rosserial_msgs::Log_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosserial_msgs::Log_<ContainerAllocator> const> ConstPtr;

}; // struct Log_

typedef ::rosserial_msgs::Log_<std::allocator<void> > Log;

typedef boost::shared_ptr< ::rosserial_msgs::Log > LogPtr;
typedef boost::shared_ptr< ::rosserial_msgs::Log const> LogConstPtr;

// constants requiring out of line definition

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosserial_msgs::Log_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosserial_msgs::Log_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rosserial_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rosserial_msgs': ['/home/eecs106a/PianoROS/project_ws/src/rosserial/rosserial_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rosserial_msgs::Log_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosserial_msgs::Log_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosserial_msgs::Log_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosserial_msgs::Log_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosserial_msgs::Log_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosserial_msgs::Log_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosserial_msgs::Log_<ContainerAllocator> >
{
  static const char* value()
  {
    return "11abd731c25933261cd6183bd12d6295";
  }

  static const char* value(const ::rosserial_msgs::Log_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x11abd731c2593326ULL;
  static const uint64_t static_value2 = 0x1cd6183bd12d6295ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosserial_msgs::Log_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosserial_msgs/Log";
  }

  static const char* value(const ::rosserial_msgs::Log_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosserial_msgs::Log_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
#ROS Logging Levels\n\
uint8 ROSDEBUG=0\n\
uint8 INFO=1\n\
uint8 WARN=2\n\
uint8 ERROR=3\n\
uint8 FATAL=4\n\
\n\
uint8 level\n\
string msg\n\
";
  }

  static const char* value(const ::rosserial_msgs::Log_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosserial_msgs::Log_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.level);
      stream.next(m.msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Log_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosserial_msgs::Log_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosserial_msgs::Log_<ContainerAllocator>& v)
  {
    s << indent << "level: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.level);
    s << indent << "msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSSERIAL_MSGS_MESSAGE_LOG_H
