#include "analysis.h"

mcap::Timestamp now()
{
  return mcap::Timestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

template <typename T>
void saveInMCAP(const std::vector<T>& messages, const std::string& outputDir, const std::string& label)
{
  // timer after here
  Timer t(outputDir + "/mcap-" + label + ".csv");
  mcap::McapWriter writer;
  auto status = writer.open(outputDir + "/mcap/" + label + ".mcap", mcap::McapWriterOptions("ros1"));

  // TODO change we get the message name via introsepction?
  mcap::Schema imageSchema("sensor_msgs/CompressedImage", "ros1msg", ros::message_traits::Definition<T>::value());

  writer.addSchema(imageSchema);

  mcap::Channel channel("/camera_c/depth/image_rect_raw", "ros1", imageSchema.id);
  writer.addChannel(channel);

  for (T message : messages)
  {
    size_t serial_size = ros::serialization::serializationLength(message);

    boost::shared_array<unsigned char> buffer(new unsigned char[serial_size]);

    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream, message);

    mcap::Message mcap_msg;
    mcap_msg.channelId = channel.id;
    mcap_msg.logTime = now();
    mcap_msg.publishTime = mcap_msg.logTime;
    mcap_msg.data = reinterpret_cast<std::byte*>(buffer.get());
    mcap_msg.dataSize = serial_size;
    auto write_status = writer.write(mcap_msg);
  }
  writer.close();
}

template <typename T>
void saveInHdf5(const std::vector<T>& messages, const std::string& outputDir, const std::string& label)
{
  // timer after here
  Timer t(outputDir + "/hdf5-" + label + ".csv");
  std::shared_ptr<std::mutex> writeMutex = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> file =
      std::make_shared<HighFive::File>(outputDir + "/hdf5/" + label + ".hdf5",
                                       HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  seerep_hdf5_ros::Hdf5Ros hdf5RosIO(file, writeMutex, "performance-test", "map");

  for (T message : messages)
  {
    hdf5RosIO.saveMessage(message);
  }
}

/**
 * 1. Argument: Path to the file to use for the data field
 * 2. Argument: Label with the current config
 * 3. Argument: Output base directory
 * 4. Argument: message size in bytes
 * 5. Argument: Total number of bytes to write
 */
int main(int argc, char** argv)
{
  if (argc < 6)
  {
    std::cout << "Missings required arguments" << std::endl;
    return -1;
  }

  ros::Time::init();
  std::vector<unsigned char> data = loadData(argv[1]);

  const std::string label = argv[2];
  const std::string outputDir = argv[3];

  Config config;
  config.messageSize = std::strtol(argv[4], 0, 10);
  config.totalSize = std::strtol(argv[5], 0, 10);

  auto messages = generateMessages(config, data);

  saveInMCAP(messages, outputDir, label);
  saveInHdf5(messages, outputDir, label);
}
