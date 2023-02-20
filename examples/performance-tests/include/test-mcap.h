#ifndef MCAP_TEST_H
#define MCAP_TEST_H

#define MCAP_IMPLEMENTATION  // Define this in exactly one .cpp file

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <chrono>
#include <mcap/writer.hpp>
#include <string>

#include "message-generation.h"
#include "timer.h"

constexpr std::string_view TOPIC = "/camera_c/depth/image_rect_raw";
constexpr std::string_view OUT_DIR = "/seerep/seerep-data/output.mcap";
// TODO: autogenerate this
constexpr std::string_view desc =
    "# This message contains an uncompressed image\n# (0, 0) is at top-left corner of image\n#\n\nHeader header        "
    "# Header timestamp should be acquisition time of image\n                     # Header frame_id should be optical "
    "frame of camera\n                     # origin of frame should be optical center of camera\n                     "
    "# +x should point to the right in the image\n                     # +y should point down in the image\n           "
    "          # +z should point into to plane of the image\n                     # If the frame_id here and the "
    "frame_id of the CameraInfo\n                     # message associated with the image conflict\n                   "
    "  # the behavior is undefined\n\nuint32 height         # image height, that is, number of rows\nuint32 width      "
    "    # image width, that is, number of columns\n\n# The legal values for encoding are in file "
    "src/image_encodings.cpp\n# If you want to standardize a new string format, join\n# "
    "ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\nstring encoding       # Encoding "
    "of pixels -- channel meaning, ordering, size\n                      # taken from the list of strings in "
    "include/sensor_msgs/image_encodings.h\n\nuint8 is_bigendian    # is this data bigendian?\nuint32 step           # "
    "Full row length in bytes\nuint8[] data          # actual matrix data, size is (step * "
    "rows)\n\n================================================================================\nMSG: "
    "std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to "
    "communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing "
    "ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch "
    "(in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable "
    "is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is "
    "associated with\nstring frame_id\n";

mcap::Timestamp now();

template <typename T>
void saveMessages(const std::vector<T>& messages);

#endif  // MCAP_TEST_H
