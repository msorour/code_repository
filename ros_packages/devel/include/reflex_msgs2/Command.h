// Generated by gencpp from file reflex_msgs2/Command.msg
// DO NOT EDIT!


#ifndef REFLEX_MSGS2_MESSAGE_COMMAND_H
#define REFLEX_MSGS2_MESSAGE_COMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <reflex_msgs2/PoseCommand.h>
#include <reflex_msgs2/VelocityCommand.h>

namespace reflex_msgs2
{
template <class ContainerAllocator>
struct Command_
{
  typedef Command_<ContainerAllocator> Type;

  Command_()
    : pose()
    , velocity()  {
    }
  Command_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , velocity(_alloc)  {
  (void)_alloc;
    }



   typedef  ::reflex_msgs2::PoseCommand_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::reflex_msgs2::VelocityCommand_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;





  typedef boost::shared_ptr< ::reflex_msgs2::Command_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reflex_msgs2::Command_<ContainerAllocator> const> ConstPtr;

}; // struct Command_

typedef ::reflex_msgs2::Command_<std::allocator<void> > Command;

typedef boost::shared_ptr< ::reflex_msgs2::Command > CommandPtr;
typedef boost::shared_ptr< ::reflex_msgs2::Command const> CommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reflex_msgs2::Command_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reflex_msgs2::Command_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::reflex_msgs2::Command_<ContainerAllocator1> & lhs, const ::reflex_msgs2::Command_<ContainerAllocator2> & rhs)
{
  return lhs.pose == rhs.pose &&
    lhs.velocity == rhs.velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::reflex_msgs2::Command_<ContainerAllocator1> & lhs, const ::reflex_msgs2::Command_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace reflex_msgs2

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::reflex_msgs2::Command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflex_msgs2::Command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflex_msgs2::Command_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflex_msgs2::Command_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflex_msgs2::Command_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflex_msgs2::Command_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reflex_msgs2::Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bcad31578e17a6697c2483ccda6d52eb";
  }

  static const char* value(const ::reflex_msgs2::Command_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbcad31578e17a669ULL;
  static const uint64_t static_value2 = 0x7c2483ccda6d52ebULL;
};

template<class ContainerAllocator>
struct DataType< ::reflex_msgs2::Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reflex_msgs2/Command";
  }

  static const char* value(const ::reflex_msgs2::Command_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reflex_msgs2::Command_<ContainerAllocator> >
{
  static const char* value()
  {
    return "PoseCommand pose\n"
"VelocityCommand velocity\n"
"\n"
"================================================================================\n"
"MSG: reflex_msgs2/PoseCommand\n"
"# Position in radians of various motors\n"
"float64 f1\n"
"float64 f2\n"
"float64 f3\n"
"float64 preshape\n"
"\n"
"================================================================================\n"
"MSG: reflex_msgs2/VelocityCommand\n"
"# Velocity in radians/second of various motors\n"
"float64 f1\n"
"float64 f2\n"
"float64 f3\n"
"float64 preshape\n"
;
  }

  static const char* value(const ::reflex_msgs2::Command_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reflex_msgs2::Command_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Command_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reflex_msgs2::Command_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reflex_msgs2::Command_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::reflex_msgs2::PoseCommand_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::reflex_msgs2::VelocityCommand_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REFLEX_MSGS2_MESSAGE_COMMAND_H
