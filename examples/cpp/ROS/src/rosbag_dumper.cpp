#include "seerep_ros_examples/rosbag_dumper.h"

namespace seerep_ros_examples
{
RosbagDumper::RosbagDumper(const std::filesystem::path& bag_path, const std::filesystem::path& hdf5_path,
                           const std::string& project_name, const std::string& project_frame)
{
  auto write_mutex = std::make_shared<std::mutex>();

  /* handle hdf5 file */
  if (std::filesystem::exists(hdf5_path))
  {
    project_uuid_ = boost::uuids::to_string(boost::uuids::random_generator()());
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

const std::vector<string_pair> RosbagDumper::matchTopics(const std::vector<std::string>& first_topics,
                                                         const std::vector<std::string>& second_topics)
{
  std::vector<string_pair> matched_topics;
  for (const std::string& first_topic : first_topics)
  {
    for (const std::string& second_topic : second_topics)
    {
      if (matchingTopics(first_topic, second_topic))
      {
        matched_topics.push_back(std::make_pair(first_topic, second_topic));
      }
    }
  }
  return matched_topics;
}

void RosbagDumper::iterateAndDumpTf(const std::string& tf_topic, const bool is_static)
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

bool RosbagDumper::matchingTopics(std::string first_topic, std::string second_topic)
{
  /* Note: find_nth is zero indexed */
  boost::iterator_range<std::string::iterator> first_it = boost::find_nth(first_topic, "/", 2);
  const std::string first_cut_str(first_topic.substr(0, std::distance(first_topic.begin(), first_it.begin())));

  boost::iterator_range<std::string::iterator> second_it = boost::find_nth(second_topic, "/", 2);
  const std::string second_cut_str(second_topic.substr(0, std::distance(second_topic.begin(), second_it.begin())));

  return first_cut_str == second_cut_str;
}

/*  TODO: add support for uncompressed images */
void RosbagDumper::iterateAndDumpCompressedImage(const std::string& image_topic, const std::string& camera_info_topic,
                                                 double viewing_distance)
{
  sensor_msgs::CameraInfo::ConstPtr last_camera_info_msg;
  std::string camera_instrinsic_uuid;

  for (const rosbag::MessageInstance& m :
       rosbag::View(bag_, rosbag::TopicQuery(std::vector<std::string>{ image_topic, camera_info_topic })))
  {
    sensor_msgs::CameraInfo::ConstPtr camera_info_msg = m.instantiate<sensor_msgs::CameraInfo>();

    if (camera_info_msg != nullptr)
    {
      /* check if we need to add another camera_intrinsic */
      if (last_camera_info_msg != camera_info_msg)
      {
        camera_instrinsic_uuid = boost::uuids::to_string(boost::uuids::random_generator()());
        /* TODO: replace with seerep_hdf5_ros interface */
        auto ci = seerep_core_fb::CoreFbConversion::fromFb(
            *seerep_ros_conversions_fb::toFlat(*camera_info_msg, project_uuid_, camera_instrinsic_uuid, viewing_distance)
                 .GetRoot());
        camera_intrinsics_io_->writeCameraIntrinsics(ci);
      }
      last_camera_info_msg = camera_info_msg;
    }

    sensor_msgs ::CompressedImage::ConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();
    if (image_msg != nullptr)
    {
      /* try to convert compressed image to image message */
      sensor_msgs::Image uncompressed_image;
      try
      {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->toImageMsg(uncompressed_image);
      }
      catch (cv::Exception& e)
      {
        ROS_ERROR_STREAM("Error while converting compressed image: " << e.what());
        /* Do not try to save the image */
        return;
      }
      /* TODO: replace with seerep_hdf5_ros interface*/
      auto image_msg = seerep_ros_conversions_fb::toFlat(uncompressed_image, project_uuid_, camera_instrinsic_uuid);
      image_io_->writeImage(boost::uuids::to_string(boost::uuids::random_generator()()), *image_msg.GetRoot());
    }
  }
}

}  // namespace seerep_ros_examples

int main(int argc, char** argv)
{
  /* ANSI escape codes for changing the color */
  const std::string GREEN_COLOR = "\033[32m";
  const std::string RESET_COLOR = "\033[0m";

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
    seerep_ros_examples::RosbagDumper dumper(bag_path, hdf5_path, project_name, project_frame);

    ROS_INFO_STREAM("---- Dumping TF messages ----");
    dumper.iterateAndDumpTf(tf_topic);

    ROS_INFO_STREAM("---- Dumping TF static messages ----");
    dumper.iterateAndDumpTf(tf_static_topic, true);

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

    ROS_INFO_STREAM("---- Dumping image messages ----");

    const std::vector<seerep_ros_examples::string_pair> matched_topics =
        dumper.matchTopics(image_topics, camera_info_topics);
    for (const seerep_ros_examples::string_pair& matched_topic : matched_topics)
    {
      std::cout << "Dumping image topic: " << matched_topic.first << std::endl;
      std::cout << "Dumping camera info topic: " << matched_topic.second << std::endl;
      dumper.iterateAndDumpCompressedImage(matched_topic.first, matched_topic.second, viewing_distance);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Missing required parameter in launch file");
    ros::shutdown();
  }

  ROS_INFO_STREAM(GREEN_COLOR << "Finished" << RESET_COLOR);
  ros::shutdown();
}
