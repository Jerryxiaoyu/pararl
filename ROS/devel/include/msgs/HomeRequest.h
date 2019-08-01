// Generated by gencpp from file msgs/HomeRequest.msg
// DO NOT EDIT!


#ifndef MSGS_MESSAGE_HOMEREQUEST_H
#define MSGS_MESSAGE_HOMEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace msgs
{
template <class ContainerAllocator>
struct HomeRequest_
{
  typedef HomeRequest_<ContainerAllocator> Type;

  HomeRequest_()
    : home(0)  {
    }
  HomeRequest_(const ContainerAllocator& _alloc)
    : home(0)  {
  (void)_alloc;
    }



   typedef int8_t _home_type;
  _home_type home;





  typedef boost::shared_ptr< ::msgs::HomeRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msgs::HomeRequest_<ContainerAllocator> const> ConstPtr;

}; // struct HomeRequest_

typedef ::msgs::HomeRequest_<std::allocator<void> > HomeRequest;

typedef boost::shared_ptr< ::msgs::HomeRequest > HomeRequestPtr;
typedef boost::shared_ptr< ::msgs::HomeRequest const> HomeRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msgs::HomeRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msgs::HomeRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'HasHeader': False, 'IsFixedSize': True, 'IsMessage': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'msgs': ['/home/hu/ProjectOtter/pararl_2/ROS/src/otter_kinova_grasping/msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct HasHeader< ::msgs::HomeRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs::HomeRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs::HomeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs::HomeRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs::HomeRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs::HomeRequest_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msgs::HomeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b7e2f7f747e266548e5f6d35de81285";
  }

  static const char* value(const ::msgs::HomeRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b7e2f7f747e2665ULL;
  static const uint64_t static_value2 = 0x48e5f6d35de81285ULL;
};

template<class ContainerAllocator>
struct DataType< ::msgs::HomeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msgs/HomeRequest";
  }

  static const char* value(const ::msgs::HomeRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msgs::HomeRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 home\n\
";
  }

  static const char* value(const ::msgs::HomeRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msgs::HomeRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.home);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HomeRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msgs::HomeRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msgs::HomeRequest_<ContainerAllocator>& v)
  {
    s << indent << "home: ";
    Printer<int8_t>::stream(s, indent + "  ", v.home);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSGS_MESSAGE_HOMEREQUEST_H