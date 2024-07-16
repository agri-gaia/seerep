#ifndef SEEREP_SERVER_SERVICE_CONSTANTS_H_
#define SEEREP_SERVER_SERVICE_CONSTANTS_H_

#include <sensor_msgs/distortion_models.h>

namespace seerep_server_constants
{
/** @brief constants for allowed distortion models*/
inline const std::string kCameraDistortionModels[] = {
  sensor_msgs::distortion_models::PLUMB_BOB,
  sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL,
  sensor_msgs::distortion_models::EQUIDISTANT
};
}  // namespace seerep_server_constants
#endif  // SEEREP_SERVER_SERVICE_CONSTANTS_H_
