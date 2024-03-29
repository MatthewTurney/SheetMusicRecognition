// Generated by gencpp from file project_pkg/Note.msg
// DO NOT EDIT!


#ifndef PROJECT_PKG_MESSAGE_NOTE_H
#define PROJECT_PKG_MESSAGE_NOTE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace project_pkg
{
template <class ContainerAllocator>
struct Note_
{
  typedef Note_<ContainerAllocator> Type;

  Note_()
    : key(0)
    , duration(0.0)
    , rest_before(0.0)  {
    }
  Note_(const ContainerAllocator& _alloc)
    : key(0)
    , duration(0.0)
    , rest_before(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _key_type;
  _key_type key;

   typedef float _duration_type;
  _duration_type duration;

   typedef float _rest_before_type;
  _rest_before_type rest_before;





  typedef boost::shared_ptr< ::project_pkg::Note_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::project_pkg::Note_<ContainerAllocator> const> ConstPtr;

}; // struct Note_

typedef ::project_pkg::Note_<std::allocator<void> > Note;

typedef boost::shared_ptr< ::project_pkg::Note > NotePtr;
typedef boost::shared_ptr< ::project_pkg::Note const> NoteConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::project_pkg::Note_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::project_pkg::Note_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace project_pkg

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'project_pkg': ['/home/eecs106a/SheetMusicRecognition/project_ws/src/project_pkg/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::project_pkg::Note_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::project_pkg::Note_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::project_pkg::Note_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::project_pkg::Note_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::project_pkg::Note_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::project_pkg::Note_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::project_pkg::Note_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff5d283ef6af3a9b602a9758c35d7198";
  }

  static const char* value(const ::project_pkg::Note_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff5d283ef6af3a9bULL;
  static const uint64_t static_value2 = 0x602a9758c35d7198ULL;
};

template<class ContainerAllocator>
struct DataType< ::project_pkg::Note_<ContainerAllocator> >
{
  static const char* value()
  {
    return "project_pkg/Note";
  }

  static const char* value(const ::project_pkg::Note_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::project_pkg::Note_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 key\n\
float32 duration\n\
float32 rest_before\n\
";
  }

  static const char* value(const ::project_pkg::Note_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::project_pkg::Note_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.key);
      stream.next(m.duration);
      stream.next(m.rest_before);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Note_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::project_pkg::Note_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::project_pkg::Note_<ContainerAllocator>& v)
  {
    s << indent << "key: ";
    Printer<int32_t>::stream(s, indent + "  ", v.key);
    s << indent << "duration: ";
    Printer<float>::stream(s, indent + "  ", v.duration);
    s << indent << "rest_before: ";
    Printer<float>::stream(s, indent + "  ", v.rest_before);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PROJECT_PKG_MESSAGE_NOTE_H
