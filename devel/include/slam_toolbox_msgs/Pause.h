// Generated by gencpp from file slam_toolbox_msgs/Pause.msg
// DO NOT EDIT!


#ifndef SLAM_TOOLBOX_MSGS_MESSAGE_PAUSE_H
#define SLAM_TOOLBOX_MSGS_MESSAGE_PAUSE_H

#include <ros/service_traits.h>


#include <slam_toolbox_msgs/PauseRequest.h>
#include <slam_toolbox_msgs/PauseResponse.h>


namespace slam_toolbox_msgs
{

struct Pause
{

typedef PauseRequest Request;
typedef PauseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Pause
} // namespace slam_toolbox_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::slam_toolbox_msgs::Pause > {
  static const char* value()
  {
    return "3a1255d4d998bd4d6585c64639b5ee9a";
  }

  static const char* value(const ::slam_toolbox_msgs::Pause&) { return value(); }
};

template<>
struct DataType< ::slam_toolbox_msgs::Pause > {
  static const char* value()
  {
    return "slam_toolbox_msgs/Pause";
  }

  static const char* value(const ::slam_toolbox_msgs::Pause&) { return value(); }
};


// service_traits::MD5Sum< ::slam_toolbox_msgs::PauseRequest> should match
// service_traits::MD5Sum< ::slam_toolbox_msgs::Pause >
template<>
struct MD5Sum< ::slam_toolbox_msgs::PauseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::slam_toolbox_msgs::Pause >::value();
  }
  static const char* value(const ::slam_toolbox_msgs::PauseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::slam_toolbox_msgs::PauseRequest> should match
// service_traits::DataType< ::slam_toolbox_msgs::Pause >
template<>
struct DataType< ::slam_toolbox_msgs::PauseRequest>
{
  static const char* value()
  {
    return DataType< ::slam_toolbox_msgs::Pause >::value();
  }
  static const char* value(const ::slam_toolbox_msgs::PauseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::slam_toolbox_msgs::PauseResponse> should match
// service_traits::MD5Sum< ::slam_toolbox_msgs::Pause >
template<>
struct MD5Sum< ::slam_toolbox_msgs::PauseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::slam_toolbox_msgs::Pause >::value();
  }
  static const char* value(const ::slam_toolbox_msgs::PauseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::slam_toolbox_msgs::PauseResponse> should match
// service_traits::DataType< ::slam_toolbox_msgs::Pause >
template<>
struct DataType< ::slam_toolbox_msgs::PauseResponse>
{
  static const char* value()
  {
    return DataType< ::slam_toolbox_msgs::Pause >::value();
  }
  static const char* value(const ::slam_toolbox_msgs::PauseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SLAM_TOOLBOX_MSGS_MESSAGE_PAUSE_H