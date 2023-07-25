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
    const std::string uuid_str = boost::uuids::to_string(boost::uuids::random_generator()());
    std::shared_ptr<HighFive::File> file =
        std::make_shared<HighFive::File>(hdf5_path.string() + "/" + uuid_str, HighFive::File::OpenOrCreate);

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
    bag_.open(bag_path.string(), rosbag::bagmode::Read);
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

/* TODO: */
void RosbagDumper::iterateAndDumpTf(const std::string& tf_topic, const bool is_static)
{
}

/* TODO: */
void RosbagDumper::iterateAndDumpImage(const std::string& image_topic, const std::string& camera_info_topic)
{
}

/* TODO: */
void RosbagDumper::dumpCameraIntrinsics(const std::string& camera_info_topic)
{
}

}  // namespace seerep_ros_examples

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_dumper");
  ros::NodeHandle nh("~");

  std::string bag_path, project_frame, project_name, hdf5_path;

  if (nh.getParam("bag_path", bag_path) && nh.getParam("project_frame", project_frame) &&
      nh.getParam("project_name", project_name) && nh.getParam("hdf5_path", hdf5_path))
  {
    seerep_ros_examples::RosbagDumper dumper(bag_path, hdf5_path, project_name, project_frame);
  }
  else
  {
    ROS_ERROR_STREAM("Missing required parameter in launch file");
    ros::shutdown();
  }
}
