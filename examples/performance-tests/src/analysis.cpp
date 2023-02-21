#include "analysis.h"

mcap::Timestamp now()
{
  return mcap::Timestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

template <typename T>
void saveInMCAP(const std::vector<T>& messages)
{
  mcap::McapWriter writer;
  auto status = writer.open(MCAP_FILE_PATH, mcap::McapWriterOptions("ros1"));

  // TODO change we get the message name via introsepction?
  mcap::Schema imageSchema("sensor_msgs/CompressedImage", "ros1msg", ros::message_traits::Definition<T>::value());

  writer.addSchema(imageSchema);

  mcap::Channel channel(TOPIC, "ros1", imageSchema.id);
  writer.addChannel(channel);

  // timer after here
  Timer t("/home/pbrstudent/Documents/seerep-data/mcap.csv");
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
void saveInHdf5(const std::vector<T>& messages)
{
  std::shared_ptr<std::mutex> writeMutex = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> file = std::make_shared<HighFive::File>(
      HDF5_FILE_PATH, HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  seerep_hdf5_ros::Hdf5Ros hdf5RosIO(file, writeMutex, "performance-test", "map");

  // timer after here
  Timer t("/home/pbrstudent/Documents/seerep-data/hdf5.csv");
  for (T message : messages)
  {
    hdf5RosIO.saveMessage(message);
  }
}

// TODO change message sizes via args
int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout << "Provide file, to use for the data field" << std::endl;
    return -1;
  }

  std::string filePath = argv[1];
  std::cout << "Using file: " << filePath << std::endl;

  Config config;
  ros::Time::init();

  std::vector<unsigned char> data = loadData(argv[1]);

  auto messages = generateMessages(config, data);
  saveInMCAP(messages);
  saveInHdf5(messages);
}
