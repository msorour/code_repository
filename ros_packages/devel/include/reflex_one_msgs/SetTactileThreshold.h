// Generated by gencpp from file reflex_one_msgs/SetTactileThreshold.msg
// DO NOT EDIT!


#ifndef REFLEX_ONE_MSGS_MESSAGE_SETTACTILETHRESHOLD_H
#define REFLEX_ONE_MSGS_MESSAGE_SETTACTILETHRESHOLD_H

#include <ros/service_traits.h>


#include <reflex_one_msgs/SetTactileThresholdRequest.h>
#include <reflex_one_msgs/SetTactileThresholdResponse.h>


namespace reflex_one_msgs
{

struct SetTactileThreshold
{

typedef SetTactileThresholdRequest Request;
typedef SetTactileThresholdResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetTactileThreshold
} // namespace reflex_one_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::reflex_one_msgs::SetTactileThreshold > {
  static const char* value()
  {
    return "01cec83f9d223083364c730460331524";
  }

  static const char* value(const ::reflex_one_msgs::SetTactileThreshold&) { return value(); }
};

template<>
struct DataType< ::reflex_one_msgs::SetTactileThreshold > {
  static const char* value()
  {
    return "reflex_one_msgs/SetTactileThreshold";
  }

  static const char* value(const ::reflex_one_msgs::SetTactileThreshold&) { return value(); }
};


// service_traits::MD5Sum< ::reflex_one_msgs::SetTactileThresholdRequest> should match
// service_traits::MD5Sum< ::reflex_one_msgs::SetTactileThreshold >
template<>
struct MD5Sum< ::reflex_one_msgs::SetTactileThresholdRequest>
{
  static const char* value()
  {
    return MD5Sum< ::reflex_one_msgs::SetTactileThreshold >::value();
  }
  static const char* value(const ::reflex_one_msgs::SetTactileThresholdRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::reflex_one_msgs::SetTactileThresholdRequest> should match
// service_traits::DataType< ::reflex_one_msgs::SetTactileThreshold >
template<>
struct DataType< ::reflex_one_msgs::SetTactileThresholdRequest>
{
  static const char* value()
  {
    return DataType< ::reflex_one_msgs::SetTactileThreshold >::value();
  }
  static const char* value(const ::reflex_one_msgs::SetTactileThresholdRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::reflex_one_msgs::SetTactileThresholdResponse> should match
// service_traits::MD5Sum< ::reflex_one_msgs::SetTactileThreshold >
template<>
struct MD5Sum< ::reflex_one_msgs::SetTactileThresholdResponse>
{
  static const char* value()
  {
    return MD5Sum< ::reflex_one_msgs::SetTactileThreshold >::value();
  }
  static const char* value(const ::reflex_one_msgs::SetTactileThresholdResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::reflex_one_msgs::SetTactileThresholdResponse> should match
// service_traits::DataType< ::reflex_one_msgs::SetTactileThreshold >
template<>
struct DataType< ::reflex_one_msgs::SetTactileThresholdResponse>
{
  static const char* value()
  {
    return DataType< ::reflex_one_msgs::SetTactileThreshold >::value();
  }
  static const char* value(const ::reflex_one_msgs::SetTactileThresholdResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // REFLEX_ONE_MSGS_MESSAGE_SETTACTILETHRESHOLD_H
