// Generated by gencpp from file planning/CameraMsgResponse.msg
// DO NOT EDIT!


#ifndef PLANNING_MESSAGE_CAMERAMSGRESPONSE_H
#define PLANNING_MESSAGE_CAMERAMSGRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/Image.h>

namespace planning
{
template <class ContainerAllocator>
struct CameraMsgResponse_
{
  typedef CameraMsgResponse_<ContainerAllocator> Type;

  CameraMsgResponse_()
    : image()  {
    }
  CameraMsgResponse_(const ContainerAllocator& _alloc)
    : image(_alloc)  {
  (void)_alloc;
    }



   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _image_type;
  _image_type image;





  typedef boost::shared_ptr< ::planning::CameraMsgResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::planning::CameraMsgResponse_<ContainerAllocator> const> ConstPtr;

}; // struct CameraMsgResponse_

typedef ::planning::CameraMsgResponse_<std::allocator<void> > CameraMsgResponse;

typedef boost::shared_ptr< ::planning::CameraMsgResponse > CameraMsgResponsePtr;
typedef boost::shared_ptr< ::planning::CameraMsgResponse const> CameraMsgResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::planning::CameraMsgResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::planning::CameraMsgResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::planning::CameraMsgResponse_<ContainerAllocator1> & lhs, const ::planning::CameraMsgResponse_<ContainerAllocator2> & rhs)
{
  return lhs.image == rhs.image;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::planning::CameraMsgResponse_<ContainerAllocator1> & lhs, const ::planning::CameraMsgResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace planning

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::planning::CameraMsgResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planning::CameraMsgResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::planning::CameraMsgResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::planning::CameraMsgResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planning::CameraMsgResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planning::CameraMsgResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::planning::CameraMsgResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b13d2865c5af2a64e6e30ab1b56e1dd5";
  }

  static const char* value(const ::planning::CameraMsgResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb13d2865c5af2a64ULL;
  static const uint64_t static_value2 = 0xe6e30ab1b56e1dd5ULL;
};

template<class ContainerAllocator>
struct DataType< ::planning::CameraMsgResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "planning/CameraMsgResponse";
  }

  static const char* value(const ::planning::CameraMsgResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::planning::CameraMsgResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensor_msgs/Image image\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/Image\n"
"# This message contains an uncompressed image\n"
"# (0, 0) is at top-left corner of image\n"
"#\n"
"\n"
"Header header        # Header timestamp should be acquisition time of image\n"
"                     # Header frame_id should be optical frame of camera\n"
"                     # origin of frame should be optical center of camera\n"
"                     # +x should point to the right in the image\n"
"                     # +y should point down in the image\n"
"                     # +z should point into to plane of the image\n"
"                     # If the frame_id here and the frame_id of the CameraInfo\n"
"                     # message associated with the image conflict\n"
"                     # the behavior is undefined\n"
"\n"
"uint32 height         # image height, that is, number of rows\n"
"uint32 width          # image width, that is, number of columns\n"
"\n"
"# The legal values for encoding are in file src/image_encodings.cpp\n"
"# If you want to standardize a new string format, join\n"
"# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n"
"\n"
"string encoding       # Encoding of pixels -- channel meaning, ordering, size\n"
"                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n"
"\n"
"uint8 is_bigendian    # is this data bigendian?\n"
"uint32 step           # Full row length in bytes\n"
"uint8[] data          # actual matrix data, size is (step * rows)\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::planning::CameraMsgResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::planning::CameraMsgResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.image);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CameraMsgResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::planning::CameraMsgResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::planning::CameraMsgResponse_<ContainerAllocator>& v)
  {
    s << indent << "image: ";
    s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.image);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLANNING_MESSAGE_CAMERAMSGRESPONSE_H
