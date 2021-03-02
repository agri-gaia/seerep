#ifndef AGRI_GAIA_ROS_CONVERSIONS
#define AGRI_GAIA_ROS_CONVERSIONS

#include <std_msgs/Header.h>
#include "Header.pb.h"


namespace agri_gaia_ros
{

  ag::ros::Header toProto(std_msgs::Header& header);



} /* namespace agri_gaia_ros */

#endif /* AGRI_GAIA_ROS_CONVERSIONS */