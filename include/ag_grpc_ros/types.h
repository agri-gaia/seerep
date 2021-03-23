#ifndef AG_GRPC_ROS_TYPES_H_
#define AG_GRPC_ROS_TYPES_H_

#include <string>
#include <algorithm>

// Supported ROS message types
#define AG_MESSAGE_TYPES(M) \
  AG_M(sensor_msgs_PointCloud2), \
  AG_M(std_msgs_Header)

namespace ag_grpc_ros{

// Create enum types
#define AG_M(e) e
enum AG_MESSAGE_TYPE { AG_MESSAGE_TYPES(M), NumTypes };
#undef AG_M

// Create the strings
#define AG_M(s) #s
  std::string MessageTypeNames[] = { AG_MESSAGE_TYPES(M) };
#undef AG_M

// Create switch case
//#define AG_M(e) case e: send()
//namespace ag_grppc_ros{
//}

std::string names()
{
  std::string s;
  for(auto name : MessageTypeNames)
    s += name + ", ";
  return s;
}

AG_MESSAGE_TYPE type( std::string name )
{
  std::replace( name.begin(), name.end(), '/', '_');
  return AG_MESSAGE_TYPE( std::find( MessageTypeNames, MessageTypeNames + NumTypes, name ) - MessageTypeNames );
}

std::string name( AG_MESSAGE_TYPE type )
{
  return (type < NumTypes) ? MessageTypeNames[ type ] : "";
}

}

#endif // AG_GRPC_ROS_TYPES_H_
