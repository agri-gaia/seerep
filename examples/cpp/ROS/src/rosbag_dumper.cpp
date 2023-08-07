#include "seerep_ros_examples/rosbag_dumper.h"

namespace seerep_ros_examples
{
RosbagDumper::RosbagDumper(const std::filesystem::path& bag_path, const std::filesystem::path& hdf5_path,
                           const std::string& project_name, const std::string& project_frame,
                           const std::string& project_uuid)
  : project_uuid_(project_uuid)
{
  auto write_mutex = std::make_shared<std::mutex>();

  /* disable boost log output for the dumper */
  boost::log::core::get()->set_logging_enabled(false);

  /* handle hdf5 file */
  if (std::filesystem::exists(hdf5_path))
  {
    std::shared_ptr<HighFive::File> file = std::make_shared<HighFive::File>(
        hdf5_path.string() + "/" + project_uuid_ + ".h5", HighFive::File::OpenOrCreate);

    /* create io interfaces based on the hdf5 file*/
    tf_io_ = std::make_shared<seerep_hdf5_fb::Hdf5FbTf>(file, write_mutex);
    image_io_ = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(file, write_mutex);
    camera_intrinsics_io_ = std::make_shared<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>(file, write_mutex);
    general_io_ = std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(file, write_mutex);

    /* write general attributes to the file */
    general_io_->writeProjectname(project_name);
    general_io_->writeProjectFrameId(project_frame);
  }
  else
  {
    ROS_ERROR_STREAM("Path to store HDF5 file does not exist: " << hdf5_path);
    ros::shutdown();
  }

  if (std::filesystem::exists(bag_path))
  {
    try
    {
      bag_.open(bag_path.string(), rosbag::bagmode::Read);
    }
    catch (rosbag::BagIOException& e)
    {
      ROS_ERROR_STREAM("Error opening rosbag: " << e.what());
      ros::shutdown();
    }
  }
  else
  {
    ROS_ERROR_STREAM("Specified rosbag does not exist: " << bag_path);
    ros::shutdown();
  }
}

RosbagDumper::~RosbagDumper()
{
  bag_.close();
}

const std::vector<std::string> RosbagDumper::getAllTopics(const std::string& topic_type)
{
  rosbag::View view(bag_);
  std::vector<std::string> topics;

  for (const rosbag::ConnectionInfo* info : view.getConnections())
  {
    if (info->datatype == topic_type)
    {
      topics.push_back(info->topic);
    }
  }
  return topics;
}

void RosbagDumper::dumpTf(const std::string& tf_topic, const bool is_static)
{
  rosbag::View view(bag_, rosbag::TopicQuery(tf_topic));

  for (const rosbag::MessageInstance& m : rosbag::View(bag_, rosbag::TopicQuery(tf_topic)))
  {
    tf2_msgs::TFMessage::ConstPtr msg = m.instantiate<tf2_msgs::TFMessage>();
    if (msg != nullptr)
    {
      for (const auto& tf : msg->transforms)
      {
        /* TODO: replace with seerep_hdf5_ros interface*/
        auto tf_msg = seerep_ros_conversions_fb::toFlat(tf, project_uuid_, is_static);
        tf_io_->writeTransformStamped(*tf_msg.GetRoot());
      }
    }
  }
}

std::string RosbagDumper::getTopicBase(std::string topic) const
{
  /* Note: find_nth is zero indexed */
  boost::iterator_range<std::string::iterator> first_it = boost::find_nth(topic, "/", 2);
  return topic.substr(0, std::distance(topic.begin(), first_it.begin()));
}

void RosbagDumper::dumpCameraInfo(const std::string& camera_info_topic, double viewing_distance)
{
  sensor_msgs::CameraInfo last_camera_info_msg;

  for (const rosbag::MessageInstance& m : rosbag::View(bag_, rosbag::TopicQuery(camera_info_topic)))
  {
    sensor_msgs::CameraInfo::ConstPtr camera_info_msg = m.instantiate<sensor_msgs::CameraInfo>();

    if (camera_info_msg != nullptr)
    {
      if (*camera_info_msg != last_camera_info_msg)
      {
        ROS_ERROR_STREAM("Changeing the CameraInfo during a mission is not supported");
        ros::shutdown();
      }

      std::string camera_instrinsic_uuid = boost::uuids::to_string(boost::uuids::random_generator()());
      /* TODO: replace with seerep_hdf5_ros interface */
      auto ci = seerep_core_fb::CoreFbConversion::fromFb(
          *seerep_ros_conversions_fb::toFlat(*camera_info_msg, project_uuid_, camera_instrinsic_uuid, viewing_distance)
               .GetRoot());
      camera_intrinsics_io_->writeCameraIntrinsics(ci);

      /* Cache the UUIDs of CameraInfo datasets. Used when saving the related image topics */
      camera_info_map_.emplace(getTopicBase(camera_info_topic), camera_instrinsic_uuid);
    }
    last_camera_info_msg = *camera_info_msg;
  }
}

/*  TODO: add support for uncompressed images */
void RosbagDumper::dumpCompressedImage(const std::string& image_topic)
{
  for (const rosbag::MessageInstance& m : rosbag::View(bag_, rosbag::TopicQuery(image_topic)))
  {
    sensor_msgs::CompressedImage::ConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();
    if (image_msg != nullptr)
    {
      /* try to convert compressed image to image message */
      sensor_msgs::Image uncompressed_image;
      try
      {
        size_t semicolon_pos = image_msg->format.find(";");
        std::string image_encoding = image_msg->format.substr(0, semicolon_pos);

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_encoding);
        cv_ptr->toImageMsg(uncompressed_image);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR_STREAM("Error while converting compressed image: " << e.what());
        return;
      }

      if (auto search = camera_info_map_.find(getTopicBase(image_topic)); search != camera_info_map_.end())
      {
        /* TODO: replace with seerep_hdf5_ros interface*/
        auto image_msg = seerep_ros_conversions_fb::toFlat(uncompressed_image, project_uuid_, search->second);
        image_io_->writeImage(boost::uuids::to_string(boost::uuids::random_generator()()), *image_msg.GetRoot());
      }
      else
      {
        ROS_ERROR_STREAM("No CameraInfo found for image topic: " << image_topic);
      }
    }
  }
}

/* iterate over file or directory path and get all bag files */
std::set<std::filesystem::path> getAllBags(const std::string& path_str)
{
  std::filesystem::path path(path_str);
  std::set<std::filesystem::path> bags_paths;

  if (std::filesystem::is_directory(std::filesystem::path(path)))
  {
    for (const auto& entry : std::filesystem::directory_iterator(path))
    {
      if (entry.is_regular_file() && entry.path().extension() == ".bag")
      {
        bags_paths.insert(entry.path());
      }
    }
  }
  else
  {
    bags_paths.insert(path);
  }
  return bags_paths;
}

}  // namespace seerep_ros_examples

/* TODO: refactor main method it's way to long */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_dumper");
  ros::NodeHandle nh("~");

  std::string bag_path, project_frame, project_name, hdf5_path, tf_topic, tf_static_topic;
  bool capture_all_image_topics;
  /* Note: Assumption that the viewing distance is the same for all cameras*/
  double viewing_distance;
  /* Ideally this should be a set, but getParam can't handle sets */
  std::vector<std::string> image_topics, camera_info_topics;

  /* TODO: write a template for this, I hate this if */
  if (nh.getParam("bag_path", bag_path) && nh.getParam("project_frame", project_frame) &&
      nh.getParam("project_name", project_name) && nh.getParam("hdf5_path", hdf5_path) &&
      nh.getParam("tf_topic", tf_topic) && nh.getParam("tf_static_topic", tf_static_topic) &&
      nh.getParam("capture_all_image_topics", capture_all_image_topics) &&
      nh.getParam("viewing_distance", viewing_distance))
  {
    const std::string project_uuid = boost::uuids::to_string(boost::uuids::random_generator()());
    for (const std::filesystem::path& bag : seerep_ros_examples::getAllBags(bag_path))
    {
      ROS_INFO_STREAM("Processing bag: " << bag.stem().string());
      seerep_ros_examples::RosbagDumper dumper(bag, hdf5_path, project_name, project_frame, project_uuid);

      dumper.dumpTf(tf_topic);

      dumper.dumpTf(tf_static_topic, true);

      if (capture_all_image_topics)
      {
        image_topics = dumper.getAllTopics("sensor_msgs/CompressedImage");
        camera_info_topics = dumper.getAllTopics("sensor_msgs/CameraInfo");
      }
      else
      {
        nh.getParam("capture_topics", image_topics);
        nh.getParam("camera_info_topics", camera_info_topics);
      }

      for (const std::string& image_topic : camera_info_topics)
      {
        dumper.dumpCameraInfo(image_topic, viewing_distance);
      }

      for (const std::string& image_topic : image_topics)
      {
        dumper.dumpCompressedImage(image_topic);
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM("Missing required parameter in launch file");
    ros::shutdown();
  }

  ros::shutdown();
}
