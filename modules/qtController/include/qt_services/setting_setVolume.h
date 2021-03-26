// Generated by gencpp from file qt_robot_interface/setting_setVolume.msg
// DO NOT EDIT!


#ifndef QT_ROBOT_INTERFACE_MESSAGE_SETTING_SETVOLUME_H
#define QT_ROBOT_INTERFACE_MESSAGE_SETTING_SETVOLUME_H

#include <ros/service_traits.h>


#include <qt_services/setting_setVolumeRequest.h>
#include <qt_services/setting_setVolumeResponse.h>


namespace qt_robot_interface
{

struct setting_setVolume
{

typedef setting_setVolumeRequest Request;
typedef setting_setVolumeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct setting_setVolume
} // namespace qt_robot_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::qt_robot_interface::setting_setVolume > {
  static const char* value()
  {
    return "14f0bfd003d9dc3318a211b307c7e7ab";
  }

  static const char* value(const ::qt_robot_interface::setting_setVolume&) { return value(); }
};

template<>
struct DataType< ::qt_robot_interface::setting_setVolume > {
  static const char* value()
  {
    return "qt_robot_interface/setting_setVolume";
  }

  static const char* value(const ::qt_robot_interface::setting_setVolume&) { return value(); }
};


// service_traits::MD5Sum< ::qt_robot_interface::setting_setVolumeRequest> should match 
// service_traits::MD5Sum< ::qt_robot_interface::setting_setVolume > 
template<>
struct MD5Sum< ::qt_robot_interface::setting_setVolumeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::qt_robot_interface::setting_setVolume >::value();
  }
  static const char* value(const ::qt_robot_interface::setting_setVolumeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_robot_interface::setting_setVolumeRequest> should match 
// service_traits::DataType< ::qt_robot_interface::setting_setVolume > 
template<>
struct DataType< ::qt_robot_interface::setting_setVolumeRequest>
{
  static const char* value()
  {
    return DataType< ::qt_robot_interface::setting_setVolume >::value();
  }
  static const char* value(const ::qt_robot_interface::setting_setVolumeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::qt_robot_interface::setting_setVolumeResponse> should match 
// service_traits::MD5Sum< ::qt_robot_interface::setting_setVolume > 
template<>
struct MD5Sum< ::qt_robot_interface::setting_setVolumeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::qt_robot_interface::setting_setVolume >::value();
  }
  static const char* value(const ::qt_robot_interface::setting_setVolumeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_robot_interface::setting_setVolumeResponse> should match 
// service_traits::DataType< ::qt_robot_interface::setting_setVolume > 
template<>
struct DataType< ::qt_robot_interface::setting_setVolumeResponse>
{
  static const char* value()
  {
    return DataType< ::qt_robot_interface::setting_setVolume >::value();
  }
  static const char* value(const ::qt_robot_interface::setting_setVolumeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QT_ROBOT_INTERFACE_MESSAGE_SETTING_SETVOLUME_H