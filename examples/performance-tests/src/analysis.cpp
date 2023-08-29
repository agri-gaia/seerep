#include "analysis.h"

mcap::Timestamp now()
{
  return mcap::Timestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

std::pair<std::chrono::nanoseconds, double> summerize_run(std::vector<std::chrono::nanoseconds>& durations,
                                                          std::vector<double>& written_bytes)
{
  return std::make_pair(std::accumulate(durations.begin(), durations.end(), std::chrono::nanoseconds(0)),
                        std::accumulate(written_bytes.begin(), written_bytes.end(), 0.0));
}

void save_run(std::pair<std::chrono::nanoseconds, double> summerized_run, const std::string& csv_path)
{
  bool write_header = false;
  std::ifstream in_file(csv_path);
  if (in_file.peek() == std::ifstream::traits_type::eof())
  {
    write_header = true;
  }
  in_file.close();

  std::ofstream out_file;
  out_file.open(csv_path, std::fstream::app);
  if (out_file.is_open())
  {
    if (write_header)
    {
      out_file << "write_ns,written_bytes\n";
    }
    out_file << summerized_run.first.count() << "," << summerized_run.second << "\n";
  }
  else
  {
    throw std::runtime_error("Could not open file " + csv_path);
  }
}

template <typename T>
void saveInMCAP(const std::vector<T>& messages, const std::string& outputDir, const std::string& label)
{
  mcap::McapWriter writer;

  mcap::McapWriterOptions options("ros1");
  // options.chunkSize = 1024 * 1;
  // options.noChunking = true;
  options.compression = mcap::Compression::None;
  // options.compressionLevel = mcap::CompressionLevel::Default;

  auto status = writer.open(outputDir + "/mcap/" + label + ".mcap", options);
  if (!status.ok())
  {
    std::cerr << "Failed to open mcap file: " << status.message << std::endl;
    return;
  }

  // add message schema
  mcap::Schema imageSchema(ros::message_traits::datatype(messages[0]), "ros1msg",
                           ros::message_traits::Definition<T>::value());
  writer.addSchema(imageSchema);

  // add channnel to write to, name does not matter
  mcap::Channel channel("/default", "ros1", imageSchema.id);
  writer.addChannel(channel);

  // serialize messages before writing
  std::vector<std::vector<uint8_t>> serialized_msgs;
  for (T message : messages)
  {
    std::vector<uint8_t> serialized_msg;
    size_t serial_size = ros::serialization::serializationLength(message);
    serialized_msg.resize(serial_size);

    ros::serialization::OStream stream(&serialized_msg[0], serial_size);
    ros::serialization::serialize(stream, message);

    serialized_msgs.push_back(serialized_msg);
  }

  std::vector<std::chrono::nanoseconds> durations;
  std::vector<double> written_bytes;

  durations.reserve(serialized_msgs.size());
  written_bytes.reserve(serialized_msgs.size());

  for (std::vector<uint8_t> buffer : serialized_msgs)
  {
    mcap::Message mcap_msg;
    mcap_msg.channelId = channel.id;
    mcap_msg.logTime = now();
    mcap_msg.publishTime = mcap_msg.logTime;
    mcap_msg.data = reinterpret_cast<std::byte*>(buffer.data());
    mcap_msg.dataSize = buffer.size();

    auto start_time = hrc::now();
    auto write_status = writer.write(mcap_msg);
    auto duration = hrc::now() - start_time;
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
    durations.push_back(duration_ns);
    written_bytes.push_back(buffer.size());
  }
  writer.close();

  save_run(summerize_run(durations, written_bytes), outputDir + "/mcap-" + label + ".csv");
}

template <typename T>
void saveInHdf5(const std::vector<T>& messages, const std::string& outputDir, const std::string& label)
{
  // timer after here
  std::shared_ptr<std::mutex> writeMutex = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> file =
      std::make_shared<HighFive::File>(outputDir + "/hdf5/" + label + ".hdf5",
                                       HighFive::File::ReadWrite | HighFive::File::Create | HighFive::File::Truncate);
  seerep_hdf5_ros::Hdf5Ros hdf5RosIO(file, writeMutex, "performance-test", "map");

  std::vector<std::chrono::nanoseconds> durations;
  std::vector<double> written_bytes;
  durations.reserve(messages.size());
  written_bytes.reserve(messages.size());

  for (T message : messages)
  {
    auto start_time = hrc::now();
    hdf5RosIO.saveMessage(message);
    auto duration = hrc::now() - start_time;
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
    durations.push_back(duration_ns);
    written_bytes.push_back(message.data.size());
  }

  save_run(summerize_run(durations, written_bytes), outputDir + "/hdf5-" + label + ".csv");
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
