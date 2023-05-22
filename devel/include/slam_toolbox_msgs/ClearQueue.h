// Generated by gencpp from file slam_toolbox_msgs/ClearQueue.msg
// DO NOT EDIT!


#ifndef SLAM_TOOLBOX_MSGS_MESSAGE_CLEARQUEUE_H
#define SLAM_TOOLBOX_MSGS_MESSAGE_CLEARQUEUE_H

#include <ros/service_traits.h>


#include <slam_toolbox_msgs/ClearQueueRequest.h>
#include <slam_toolbox_msgs/ClearQueueResponse.h>


namespace slam_toolbox_msgs
{

struct ClearQueue
{

typedef ClearQueueRequest Request;
typedef ClearQueueResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ClearQueue
} // namespace slam_toolbox_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::slam_toolbox_msgs::ClearQueue > {
  static const char* value()
  {
    return "3a1255d4d998bd4d6585c64639b5ee9a";
  }

  static const char* value(const ::slam_toolbox_msgs::ClearQueue&) { return value(); }
};

template<>
struct DataType< ::slam_toolbox_msgs::ClearQueue > {
  static const char* value()
  {
    return "slam_toolbox_msgs/ClearQueue";
  }

  static const char* value(const ::slam_toolbox_msgs::ClearQueue&) { return value(); }
};


// service_traits::MD5Sum< ::slam_toolbox_msgs::ClearQueueRequest> should match
// service_traits::MD5Sum< ::slam_toolbox_msgs::ClearQueue >
template<>
struct MD5Sum< ::slam_toolbox_msgs::ClearQueueRequest>
{
  static const char* value()
  {
    return MD5Sum< ::slam_toolbox_msgs::ClearQueue >::value();
  }
  static const char* value(const ::slam_toolbox_msgs::ClearQueueRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::slam_toolbox_msgs::ClearQueueRequest> should match
// service_traits::DataType< ::slam_toolbox_msgs::ClearQueue >
template<>
struct DataType< ::slam_toolbox_msgs::ClearQueueRequest>
{
  static const char* value()
  {
    return DataType< ::slam_toolbox_msgs::ClearQueue >::value();
  }
  static const char* value(const ::slam_toolbox_msgs::ClearQueueRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::slam_toolbox_msgs::ClearQueueResponse> should match
// service_traits::MD5Sum< ::slam_toolbox_msgs::ClearQueue >
template<>
struct MD5Sum< ::slam_toolbox_msgs::ClearQueueResponse>
{
  static const char* value()
  {
    return MD5Sum< ::slam_toolbox_msgs::ClearQueue >::value();
  }
  static const char* value(const ::slam_toolbox_msgs::ClearQueueResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::slam_toolbox_msgs::ClearQueueResponse> should match
// service_traits::DataType< ::slam_toolbox_msgs::ClearQueue >
template<>
struct DataType< ::slam_toolbox_msgs::ClearQueueResponse>
{
  static const char* value()
  {
    return DataType< ::slam_toolbox_msgs::ClearQueue >::value();
  }
  static const char* value(const ::slam_toolbox_msgs::ClearQueueResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SLAM_TOOLBOX_MSGS_MESSAGE_CLEARQUEUE_H