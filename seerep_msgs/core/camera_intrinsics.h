#ifndef SEEREP_CORE_MSGS_CAMERA_INTRINSICS_H_
#define SEEREP_CORE_MSGS_CAMERA_INTRINSICS_H_

#include <header.h>
#include <region_of_interest.h>

namespace seerep_core_msgs
{
struct camera_intrinsics
{
  Header header;
  uint32_t height;
  uint32_t width;
  std::string distortion_model;
  std::vector<double> distortion;
  std::vector<double> intrinsic_matrix;
  std::vector<double> rectification_matrix;
  std::vector<double> projection_matrix;
  uint32_t binning_x;
  uint32_t binning_y;
  seerep_core_msgs::region_of_interest region_of_interest;
  uint32_t maximum_viewing_distance;
};
}  // namespace seerep_core_msgs

#endif  // SEEREP_CORE_MSGS_CAMERA_INTRINSICS_H_
