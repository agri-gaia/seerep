#define MCAP_IMPLEMENTATION  // Define this in exactly one .cpp file
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <chrono>
#include <mcap/writer.hpp>
#include <string>

#include "timer.h"

constexpr std::string_view TOPIC = "/camera_c/depth/image_rect_raw";
constexpr std::string_view OUT_DIR = "/seerep/seerep-data/output.mcap";
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

mcap::Timestamp now()
{
  return mcap::Timestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

void storeCallBack(const sensor_msgs::Image::ConstPtr& msg, mcap::McapWriter* writer, mcap::Channel& channel)
{
  Timer("/seerep/seerep-data/macp_data.csv");
  // serialize the ROS message
  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, *msg);

  mcap::Message mcap_msg;
  mcap_msg.channelId = channel.id;
  mcap_msg.logTime = now();
  mcap_msg.publishTime = mcap_msg.logTime;
  mcap_msg.data = reinterpret_cast<std::byte*>(buffer.get());
  mcap_msg.dataSize = serial_size;
  auto write_status = writer->write(mcap_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MCAP Dumper");

  ros::NodeHandle nh;

  // setup macp file + channel
  mcap::McapWriter writer;
  auto status = writer.open(OUT_DIR, mcap::McapWriterOptions("ros1"));

  mcap::Schema imageSchema("sensor_msgs/Image", "ros1msg", desc);

  writer.addSchema(imageSchema);

  mcap::Channel publisher(TOPIC, "ros1", imageSchema.id);
  writer.addChannel(publisher);

  ros::Subscriber sub =
      nh.subscribe<sensor_msgs::Image>(std::string(TOPIC), 1000, boost::bind(storeCallBack, _1, &writer, publisher));

  ros::spin();
  writer.close();
}
