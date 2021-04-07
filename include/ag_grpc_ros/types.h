#ifndef AG_GRPC_ROS_TYPES_H_
#define AG_GRPC_ROS_TYPES_H_

// std
#include <string>
#include <algorithm>

// ag
#include <ag_proto_msgs/transfer_sensor_msgs.grpc.pb.h>
#include <ag_proto_ros/conversions.h>

// Supported ROS message types
#define AG_MESSAGE_TYPES(M) \
  AG_M(std_msgs, Header), \
  AG_M(sensor_msgs, PointCloud2), \
  AG_M(sensor_msgs, Image)

namespace ag_grpc_ros{

// Create enum types
#define AG_M(p, t) p ## _ ## t
enum AG_MESSAGE_TYPE { AG_MESSAGE_TYPES(M), NumTypes };
#undef AG_M

#define PPCAT_NX(A, B) A ## _ ## B
#define PPCAT(A, B) PPCAT_NX(A, B)
#define STRINGIZE_NX(A) #A
#define STRINGIZE(A) STRINGIZE_NX(A)

// Create the strings
#define STR_CONCAT (STR1 STR2
#define AG_M(p, t) STRINGIZE(PPCAT(p, t))
  std::string MessageTypeNames[] = { AG_MESSAGE_TYPES(M) };
#undef AG_M

using StubPtr = std::unique_ptr<ag::TransferSensorMsgs::Stub>;

template<typename Type>
void send(StubPtr& stub, grpc::ClientContext* c, ag::ServerResponse* r, Type msg){}

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
