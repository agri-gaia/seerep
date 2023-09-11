#include "seerep_ros_communication/rosbagConverterEval.h"

namespace seerep_grpc_ros
{
RosbagConverterEval::RosbagConverterEval(const std::string& bagPath, const std::string& hdf5FilePath,
                                         const std::string& projectFrameId, const std::string& projectName,
                                         const std::string& projectUuid, const std::string& topicImage)
  : bagPath(bagPath)
  , hdf5FilePath(hdf5FilePath)
  , projectFrameId(projectFrameId)
  , projectName(projectName)
  , projectUuid(projectUuid)
  , topicImage(topicImage)
{
  for (int i = 0; i < 50; i++)
  {
    std::filesystem::remove(hdf5FilePath);
    auto writeMtx = std::make_shared<std::mutex>();
    std::shared_ptr<HighFive::File> hdf5File =
        std::make_shared<HighFive::File>(hdf5FilePath, HighFive::File::OpenOrCreate);
    hdf5ros = std::make_shared<seerep_hdf5_ros::Hdf5Ros>(hdf5File, writeMtx, std::filesystem::path(bagPath).filename(),
                                                         "map");

    iterateAndDumpImages();
  }
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();

  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / v.size());

  std::cout << "mean: " << mean << std::endl;
  std::cout << "stddev: " << stdev << std::endl;
}

RosbagConverterEval::~RosbagConverterEval()
{
}

void RosbagConverterEval::iterateAndDumpImages()
{
  std::vector<std::string> topics;
  topics.push_back("/camera_c/color/image_rect_color");
  topics.push_back("/camera_r/color/image_rect_color");
  topics.push_back("/camera_l/color/image_rect_color");

  auto start = std::chrono::high_resolution_clock::now();

  bag.open(bagPath);

  for (auto topic : topics)
  {
    {
      for (const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topic)))
      {
        sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
        if (msg != nullptr)
        {
          // auto a = msg->data.size() * sizeof(msg->data[0]);
          // auto b = msg->encoding.size() * sizeof(msg->encoding[0]);
          // auto c = msg->header.frame_id.size() * sizeof(msg->header.frame_id[0]);
          // auto d = sizeof(msg->header.seq);
          // auto e = sizeof(msg->header.stamp.sec);
          // auto f = sizeof(msg->header.stamp.nsec);
          // auto g = sizeof(msg->height);
          // auto h = sizeof(msg->is_bigendian);
          // auto i = sizeof(msg->step);
          // auto j = sizeof(msg->width);
          // std::cout << a << std::endl;
          // std::cout << b << std::endl;
          // std::cout << c << std::endl;
          // std::cout << d << std::endl;
          // std::cout << e << std::endl;
          // std::cout << f << std::endl;
          // std::cout << g << std::endl;
          // std::cout << h << std::endl;
          // std::cout << i << std::endl;
          // std::cout << j << std::endl;
          // std::cout << a + b + c + d + e + f + g + h + i + j << std::endl;
          hdf5ros->saveMessage(*msg);
        }
        else
        {
          ROS_ERROR_STREAM("nullptr while iterating images");
        }
      }
    }
  }
  bag.close();
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << duration.count() << std::endl;
  v.push_back(duration.count());
}

}  // namespace seerep_grpc_ros

std::string getHDF5FilePath(ros::NodeHandle privateNh, std::string& projectUuid)
{
  std::string hdf5FolderPath;
  if (!privateNh.getParam("hdf5FolderPath", hdf5FolderPath))
  {
    ROS_WARN_STREAM("Use the \"hdf5FolderPath\" parameter to specify the HDF5 file!");
    return "";
  }

  if (privateNh.getParam("projectUuid", projectUuid))
  {
    try
    {
      boost::uuids::string_generator gen;
      // if this throws no exception, the UUID is valid
      gen(projectUuid);
    }
    catch (std::runtime_error const& e)
    {
      // mainly catching "invalid uuid string"
      ROS_ERROR_STREAM(e.what());
      projectUuid = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      ROS_WARN_STREAM("The provided UUID is invalid! Generating a a new one. (" + projectUuid + ".h5)");
    }
  }
  else
  {
    projectUuid = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
    ROS_WARN_STREAM("Use the \"projectUuid\" parameter to specify the HDF5 file! Generating a a new one. (" +
                    projectUuid + ".h5)");
  }

  return hdf5FolderPath + "/" + projectUuid + ".h5";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_RosbagConverterEval");
  ros::NodeHandle privateNh("~");

  boost::log::add_common_attributes();
  boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");
  boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::fatal);

  std::string bagPath, projectFrameId, projectName, topicImage;

  std::string projectUuid;
  const std::string hdf5FilePath = getHDF5FilePath(privateNh, projectUuid);

  if (privateNh.getParam("bagPath", bagPath) && privateNh.getParam("projectFrameId", projectFrameId) &&
      privateNh.getParam("projectName", projectName) && privateNh.getParam("topicImage", topicImage))
  {
    ROS_INFO_STREAM("hdf5FilePath: " << hdf5FilePath);
    ROS_INFO_STREAM("bagPath: " << bagPath);
    ROS_INFO_STREAM("projectFrameId: " << projectFrameId);
    ROS_INFO_STREAM("projectName: " << projectName);
    ROS_INFO_STREAM("topicImage: " << topicImage);

    seerep_grpc_ros::RosbagConverterEval RosbagConverterEval(bagPath, hdf5FilePath, projectFrameId, projectName,
                                                             projectUuid, topicImage);
  }
  else
  {
    ROS_ERROR_STREAM("Not all mandatory parameters are set!");
  }

  return EXIT_SUCCESS;
}
