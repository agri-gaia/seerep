#ifndef SEEREP_CORE_MSGS_REGION_OF_INTEREST_H_
#define SEEREP_CORE_MSGS_REGION_OF_INTEREST_H_

namespace seerep_core_msgs
{
struct region_of_interest
{
  uint32_t x_offset;
  uint32_t y_offset;
  uint32_t height;
  uint32_t width;
  bool do_rectify;
};
}  // namespace seerep_core_msgs

#endif  // SEEREP_CORE_MSGS_REGION_OF_INTEREST_H_
