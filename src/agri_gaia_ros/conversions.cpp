#include "agri_gaia_ros/conversions.h"

namespace agri_gaia_ros
{

  ag::ros::Header toProto(std_msgs::Header& header)
  {
    ag::ros::Header ret;
    ret.set_seq(header.seq);
    ret.set_frame_id(header.frame_id);
    ret.mutable_stamp()->set_seconds(header.stamp.sec);
    ret.mutable_stamp()->set_nanos(header.stamp.nsec);
    return ret;
  }

} /* namespace agri_gaia_ros */
