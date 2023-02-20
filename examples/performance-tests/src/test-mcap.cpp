#include "test-mcap.h"

mcap::Timestamp now()
{
  return mcap::Timestamp(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

template <typename T>
void saveMessages(const std::vector<T>& messages)
{
  // setup
  mcap::McapWriter writer;
  auto status = writer.open(OUT_DIR, mcap::McapWriterOptions("ros1"));

  mcap::Schema imageSchema("sensor_msgs/CompressedImage", "ros1msg", desc);

  writer.addSchema(imageSchema);

  mcap::Channel channel(TOPIC, "ros1", imageSchema.id);
  writer.addChannel(channel);

  {
    Timer("/seerep/seerep-data/mcap.csv");
    for (auto message : messages)
    {
      uint32_t serial_size = ros::serialization::serializationLength(message);
      boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

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
  }

  writer.close();
}

int main()
{
  Config config;
  auto messages = generateMessages(config);
  saveMessages(messages);
}
