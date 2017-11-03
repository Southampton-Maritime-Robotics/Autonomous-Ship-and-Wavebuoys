/* Auto-generated by genmsg_cpp for file /home/gdp40/fuerte_workspace/sandbox/ASV/msg/rudder.msg */
#ifndef ASV_MESSAGE_RUDDER_H
#define ASV_MESSAGE_RUDDER_H
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


namespace ASV
{
template <class ContainerAllocator>
struct rudder_ {
  typedef rudder_<ContainerAllocator> Type;

  rudder_()
  : rudder_demand(0.0)
  {
  }

  rudder_(const ContainerAllocator& _alloc)
  : rudder_demand(0.0)
  {
  }

  typedef float _rudder_demand_type;
  float rudder_demand;


  typedef boost::shared_ptr< ::ASV::rudder_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ASV::rudder_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct rudder
typedef  ::ASV::rudder_<std::allocator<void> > rudder;

typedef boost::shared_ptr< ::ASV::rudder> rudderPtr;
typedef boost::shared_ptr< ::ASV::rudder const> rudderConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ASV::rudder_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ASV::rudder_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ASV

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ASV::rudder_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ASV::rudder_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ASV::rudder_<ContainerAllocator> > {
  static const char* value() 
  {
    return "60078da9d4c3d9307b5235a0a0d7c95a";
  }

  static const char* value(const  ::ASV::rudder_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x60078da9d4c3d930ULL;
  static const uint64_t static_value2 = 0x7b5235a0a0d7c95aULL;
};

template<class ContainerAllocator>
struct DataType< ::ASV::rudder_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ASV/rudder";
  }

  static const char* value(const  ::ASV::rudder_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ASV::rudder_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 rudder_demand\n\
\n\
";
  }

  static const char* value(const  ::ASV::rudder_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ASV::rudder_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ASV::rudder_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.rudder_demand);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct rudder_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ASV::rudder_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ASV::rudder_<ContainerAllocator> & v) 
  {
    s << indent << "rudder_demand: ";
    Printer<float>::stream(s, indent + "  ", v.rudder_demand);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ASV_MESSAGE_RUDDER_H

