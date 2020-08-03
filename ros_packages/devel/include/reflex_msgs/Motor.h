// Generated by gencpp from file reflex_msgs/Motor.msg
// DO NOT EDIT!


#ifndef REFLEX_MSGS_MESSAGE_MOTOR_H
#define REFLEX_MSGS_MESSAGE_MOTOR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace reflex_msgs
{
template <class ContainerAllocator>
struct Motor_
{
  typedef Motor_<ContainerAllocator> Type;

  Motor_()
    : joint_angle(0.0)
    , raw_angle(0.0)
    , velocity(0.0)
    , load(0.0)
    , voltage(0.0)
    , temperature(0)
    , error_state()  {
    }
  Motor_(const ContainerAllocator& _alloc)
    : joint_angle(0.0)
    , raw_angle(0.0)
    , velocity(0.0)
    , load(0.0)
    , voltage(0.0)
    , temperature(0)
    , error_state(_alloc)  {
  (void)_alloc;
    }



   typedef double _joint_angle_type;
  _joint_angle_type joint_angle;

   typedef double _raw_angle_type;
  _raw_angle_type raw_angle;

   typedef double _velocity_type;
  _velocity_type velocity;

   typedef double _load_type;
  _load_type load;

   typedef double _voltage_type;
  _voltage_type voltage;

   typedef int32_t _temperature_type;
  _temperature_type temperature;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _error_state_type;
  _error_state_type error_state;





  typedef boost::shared_ptr< ::reflex_msgs::Motor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reflex_msgs::Motor_<ContainerAllocator> const> ConstPtr;

}; // struct Motor_

typedef ::reflex_msgs::Motor_<std::allocator<void> > Motor;

typedef boost::shared_ptr< ::reflex_msgs::Motor > MotorPtr;
typedef boost::shared_ptr< ::reflex_msgs::Motor const> MotorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reflex_msgs::Motor_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reflex_msgs::Motor_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::reflex_msgs::Motor_<ContainerAllocator1> & lhs, const ::reflex_msgs::Motor_<ContainerAllocator2> & rhs)
{
  return lhs.joint_angle == rhs.joint_angle &&
    lhs.raw_angle == rhs.raw_angle &&
    lhs.velocity == rhs.velocity &&
    lhs.load == rhs.load &&
    lhs.voltage == rhs.voltage &&
    lhs.temperature == rhs.temperature &&
    lhs.error_state == rhs.error_state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::reflex_msgs::Motor_<ContainerAllocator1> & lhs, const ::reflex_msgs::Motor_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace reflex_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::reflex_msgs::Motor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflex_msgs::Motor_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflex_msgs::Motor_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflex_msgs::Motor_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflex_msgs::Motor_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflex_msgs::Motor_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reflex_msgs::Motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "66d6779b4fae4b7b68e0863263c3993c";
  }

  static const char* value(const ::reflex_msgs::Motor_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x66d6779b4fae4b7bULL;
  static const uint64_t static_value2 = 0x68e0863263c3993cULL;
};

template<class ContainerAllocator>
struct DataType< ::reflex_msgs::Motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reflex_msgs/Motor";
  }

  static const char* value(const ::reflex_msgs::Motor_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reflex_msgs::Motor_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 joint_angle\n"
"float64 raw_angle\n"
"float64 velocity\n"
"float64 load\n"
"float64 voltage\n"
"int32 temperature\n"
"string error_state\n"
;
  }

  static const char* value(const ::reflex_msgs::Motor_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reflex_msgs::Motor_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_angle);
      stream.next(m.raw_angle);
      stream.next(m.velocity);
      stream.next(m.load);
      stream.next(m.voltage);
      stream.next(m.temperature);
      stream.next(m.error_state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Motor_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reflex_msgs::Motor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reflex_msgs::Motor_<ContainerAllocator>& v)
  {
    s << indent << "joint_angle: ";
    Printer<double>::stream(s, indent + "  ", v.joint_angle);
    s << indent << "raw_angle: ";
    Printer<double>::stream(s, indent + "  ", v.raw_angle);
    s << indent << "velocity: ";
    Printer<double>::stream(s, indent + "  ", v.velocity);
    s << indent << "load: ";
    Printer<double>::stream(s, indent + "  ", v.load);
    s << indent << "voltage: ";
    Printer<double>::stream(s, indent + "  ", v.voltage);
    s << indent << "temperature: ";
    Printer<int32_t>::stream(s, indent + "  ", v.temperature);
    s << indent << "error_state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.error_state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REFLEX_MSGS_MESSAGE_MOTOR_H
