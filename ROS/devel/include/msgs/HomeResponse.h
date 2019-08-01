// Generated by gencpp from file msgs/HomeResponse.msg
// DO NOT EDIT!


#ifndef MSGS_MESSAGE_HOMERESPONSE_H
#define MSGS_MESSAGE_HOMERESPONSE_H


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
struct HomeResponse_
{
  typedef HomeResponse_<ContainerAllocator> Type;

  HomeResponse_()
    : done(0)  {
    }
  HomeResponse_(const ContainerAllocator& _alloc)
    : done(0)  {
  (void)_alloc;
    }



   typedef int8_t _done_type;
  _done_type done;





  typedef boost::shared_ptr< ::msgs::HomeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msgs::HomeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct HomeResponse_

typedef ::msgs::HomeResponse_<std::allocator<void> > HomeResponse;

typedef boost::shared_ptr< ::msgs::HomeResponse > HomeResponsePtr;
typedef boost::shared_ptr< ::msgs::HomeResponse const> HomeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msgs::HomeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msgs::HomeResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct HasHeader< ::msgs::HomeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msgs::HomeResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs::HomeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msgs::HomeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs::HomeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msgs::HomeResponse_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msgs::HomeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a265a3e43e23769318ae493072a49313";
  }

  static const char* value(const ::msgs::HomeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa265a3e43e237693ULL;
  static const uint64_t static_value2 = 0x18ae493072a49313ULL;
};

template<class ContainerAllocator>
struct DataType< ::msgs::HomeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msgs/HomeResponse";
  }

  static const char* value(const ::msgs::HomeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msgs::HomeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 done\n\
";
  }

  static const char* value(const ::msgs::HomeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msgs::HomeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.done);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HomeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msgs::HomeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msgs::HomeResponse_<ContainerAllocator>& v)
  {
    s << indent << "done: ";
    Printer<int8_t>::stream(s, indent + "  ", v.done);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSGS_MESSAGE_HOMERESPONSE_H