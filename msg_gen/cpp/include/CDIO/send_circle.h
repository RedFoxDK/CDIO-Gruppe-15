/* Auto-generated by genmsg_cpp for file /home/redfox/Desktop/CDIO/msg/send_circle.msg */
#ifndef CDIO_MESSAGE_SEND_CIRCLE_H
#define CDIO_MESSAGE_SEND_CIRCLE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace CDIO
{
template <class ContainerAllocator>
struct send_circle_ {
  typedef send_circle_<ContainerAllocator> Type;

  send_circle_()
  : centerX(0.0)
  , centerY(0.0)
  , radius(0.0)
  {
  }

  send_circle_(const ContainerAllocator& _alloc)
  : centerX(0.0)
  , centerY(0.0)
  , radius(0.0)
  {
  }

  typedef double _centerX_type;
  double centerX;

  typedef double _centerY_type;
  double centerY;

  typedef double _radius_type;
  double radius;


  typedef boost::shared_ptr< ::CDIO::send_circle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::CDIO::send_circle_<ContainerAllocator>  const> ConstPtr;
}; // struct send_circle
typedef  ::CDIO::send_circle_<std::allocator<void> > send_circle;

typedef boost::shared_ptr< ::CDIO::send_circle> send_circlePtr;
typedef boost::shared_ptr< ::CDIO::send_circle const> send_circleConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::CDIO::send_circle_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::CDIO::send_circle_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace CDIO

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::CDIO::send_circle_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::CDIO::send_circle_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::CDIO::send_circle_<ContainerAllocator> > {
  static const char* value() 
  {
    return "34eb6927a044b5d4aa33e7f8dcea08d2";
  }

  static const char* value(const  ::CDIO::send_circle_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x34eb6927a044b5d4ULL;
  static const uint64_t static_value2 = 0xaa33e7f8dcea08d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::CDIO::send_circle_<ContainerAllocator> > {
  static const char* value() 
  {
    return "CDIO/send_circle";
  }

  static const char* value(const  ::CDIO::send_circle_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::CDIO::send_circle_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 centerX\n\
float64 centerY\n\
float64 radius\n\
\n\
";
  }

  static const char* value(const  ::CDIO::send_circle_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::CDIO::send_circle_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::CDIO::send_circle_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.centerX);
    stream.next(m.centerY);
    stream.next(m.radius);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct send_circle_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::CDIO::send_circle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::CDIO::send_circle_<ContainerAllocator> & v) 
  {
    s << indent << "centerX: ";
    Printer<double>::stream(s, indent + "  ", v.centerX);
    s << indent << "centerY: ";
    Printer<double>::stream(s, indent + "  ", v.centerY);
    s << indent << "radius: ";
    Printer<double>::stream(s, indent + "  ", v.radius);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CDIO_MESSAGE_SEND_CIRCLE_H

