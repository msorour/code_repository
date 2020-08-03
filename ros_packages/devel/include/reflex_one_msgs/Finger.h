// Generated by gencpp from file reflex_one_msgs/Finger.msg
// DO NOT EDIT!


#ifndef REFLEX_ONE_MSGS_MESSAGE_FINGER_H
#define REFLEX_ONE_MSGS_MESSAGE_FINGER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace reflex_one_msgs
{
template <class ContainerAllocator>
struct Finger_
{
  typedef Finger_<ContainerAllocator> Type;

  Finger_()
    : proximal(0.0)
    , distal_approx(0.0)
    , contact()
    , pressure()  {
      contact.assign(false);

      pressure.assign(0.0);
  }
  Finger_(const ContainerAllocator& _alloc)
    : proximal(0.0)
    , distal_approx(0.0)
    , contact()
    , pressure()  {
  (void)_alloc;
      contact.assign(false);

      pressure.assign(0.0);
  }



   typedef float _proximal_type;
  _proximal_type proximal;

   typedef float _distal_approx_type;
  _distal_approx_type distal_approx;

   typedef boost::array<uint8_t, 9>  _contact_type;
  _contact_type contact;

   typedef boost::array<float, 9>  _pressure_type;
  _pressure_type pressure;





  typedef boost::shared_ptr< ::reflex_one_msgs::Finger_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reflex_one_msgs::Finger_<ContainerAllocator> const> ConstPtr;

}; // struct Finger_

typedef ::reflex_one_msgs::Finger_<std::allocator<void> > Finger;

typedef boost::shared_ptr< ::reflex_one_msgs::Finger > FingerPtr;
typedef boost::shared_ptr< ::reflex_one_msgs::Finger const> FingerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reflex_one_msgs::Finger_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reflex_one_msgs::Finger_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::reflex_one_msgs::Finger_<ContainerAllocator1> & lhs, const ::reflex_one_msgs::Finger_<ContainerAllocator2> & rhs)
{
  return lhs.proximal == rhs.proximal &&
    lhs.distal_approx == rhs.distal_approx &&
    lhs.contact == rhs.contact &&
    lhs.pressure == rhs.pressure;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::reflex_one_msgs::Finger_<ContainerAllocator1> & lhs, const ::reflex_one_msgs::Finger_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace reflex_one_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::reflex_one_msgs::Finger_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflex_one_msgs::Finger_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflex_one_msgs::Finger_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflex_one_msgs::Finger_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflex_one_msgs::Finger_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflex_one_msgs::Finger_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reflex_one_msgs::Finger_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b5232f74e901b48063f64cfc32aefbe0";
  }

  static const char* value(const ::reflex_one_msgs::Finger_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb5232f74e901b480ULL;
  static const uint64_t static_value2 = 0x63f64cfc32aefbe0ULL;
};

template<class ContainerAllocator>
struct DataType< ::reflex_one_msgs::Finger_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reflex_one_msgs/Finger";
  }

  static const char* value(const ::reflex_one_msgs::Finger_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reflex_one_msgs::Finger_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# message for ReFlex Fingers\n"
"float32 proximal		# radians, measured from all open = 0, to pi = closed\n"
"float32 distal_approx   # radians, measured from all open = 0, to roughly pi = against proximal pad, relative to prox link\n"
"bool[9] contact			# binary, 0 = proximal, 8 = fingertip\n"
"float32[9] pressure		# scalar, dimensionless units, 0 = proximal, 8 = fingertip (can go negative)\n"
;
  }

  static const char* value(const ::reflex_one_msgs::Finger_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reflex_one_msgs::Finger_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.proximal);
      stream.next(m.distal_approx);
      stream.next(m.contact);
      stream.next(m.pressure);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Finger_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reflex_one_msgs::Finger_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reflex_one_msgs::Finger_<ContainerAllocator>& v)
  {
    s << indent << "proximal: ";
    Printer<float>::stream(s, indent + "  ", v.proximal);
    s << indent << "distal_approx: ";
    Printer<float>::stream(s, indent + "  ", v.distal_approx);
    s << indent << "contact[]" << std::endl;
    for (size_t i = 0; i < v.contact.size(); ++i)
    {
      s << indent << "  contact[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.contact[i]);
    }
    s << indent << "pressure[]" << std::endl;
    for (size_t i = 0; i < v.pressure.size(); ++i)
    {
      s << indent << "  pressure[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.pressure[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // REFLEX_ONE_MSGS_MESSAGE_FINGER_H
