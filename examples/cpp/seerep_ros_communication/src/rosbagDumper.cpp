#include "seerep_ros_communication/rosbagDumper.h"

namespace seerep_grpc_ros
{
RosbagDumper::RosbagDumper(std::string bagPath, std::string hdf5FilePath, std::string project_frame_id,
                           std::string project_name, std::string topicImage, std::string topicCameraIntrinsics,
                           std::string topicDetection, std::string detectionCategory, std::string topicTf,
                           std::string topicTfStatic, std::string topicGeoAnchor, float distanceCameraGround)
  : hdf5FilePath(hdf5FilePath)
  , project_frame_id(project_frame_id)
  , project_name(project_name)
  , topicImage(topicImage)
  , topicCameraIntrinsics(topicCameraIntrinsics)
  , topicDetection(topicDetection)
  , detectionCategory(detectionCategory)
  , topicTf(topicTf)
  , topicTfStatic(topicTfStatic)
  , topicGeoAnchor(topicGeoAnchor)
  , distanceCameraGround(distanceCameraGround)
{
  auto write_mtx = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> hdf5_file =
      std::make_shared<HighFive::File>(hdf5FilePath, HighFive::File::OpenOrCreate);
  m_ioCoreGeneral = std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(hdf5_file, write_mtx);
  m_ioTf = std::make_shared<seerep_hdf5_fb::Hdf5FbTf>(hdf5_file, write_mtx);
  m_ioImage = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(hdf5_file, write_mtx);
  m_ioImageCore = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(hdf5_file, write_mtx);
  m_ioPoint = std::make_shared<seerep_hdf5_fb::Hdf5FbPoint>(hdf5_file, write_mtx);

  m_ioCoreGeneral->writeProjectname(std::filesystem::path(bagPath).filename());
  m_ioCoreGeneral->writeProjectFrameId("map");

  bag.open(bagPath);

  getGeoAnchor();
  getCameraIntrinsic();
  iterateAndDumpTf();
  iterateAndDumpImages();
  iterateAndDumpDetections();
}

RosbagDumper::~RosbagDumper()
{
  bag.close();
}

void RosbagDumper::getGeoAnchor()
{
  for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topicGeoAnchor)))
  {
    geographic_msgs::GeoPointStamped::ConstPtr msg = m.instantiate<geographic_msgs::GeoPointStamped>();

    seerep_core_msgs::GeodeticCoordinates geoCoordinates;
    geoCoordinates.latitude = msg->position.latitude;
    geoCoordinates.longitude = msg->position.longitude;
    geoCoordinates.altitude = msg->position.altitude;
    geoCoordinates.ellipsoid = "";
    geoCoordinates.coordinateSystem = "";
    m_ioCoreGeneral->writeGeodeticLocation(geoCoordinates);
  }
}

void RosbagDumper::getCameraIntrinsic()
{
  for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topicCameraIntrinsics)))
  {
    sensor_msgs::CameraInfo::ConstPtr msg = m.instantiate<sensor_msgs::CameraInfo>();

    cameraInfo = *msg;
    break;
  }
}

void RosbagDumper::iterateAndDumpImages()
{
  for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topicImage)))
  {
    sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
    if (msg != nullptr)
    {
      boost::uuids::uuid msguuid = boost::uuids::random_generator()();
      std::string uuidstring = boost::lexical_cast<std::string>(msguuid);
      ROS_DEBUG_STREAM(uuidstring << " storing image"
                                  << " time: " << msg->header.stamp.sec << " / " << msg->header.stamp.nsec);
      uint64_t time = (uint64_t)msg->header.stamp.sec << 32 | msg->header.stamp.nsec;
      {  // scope of lock
        const std::scoped_lock lock(timeUuidMapMutex_);
        timeUuidMap_.emplace(time, uuidstring);
      }
      std::string projectuuiddummy = "";
      auto imgMsg = seerep_ros_conversions_fb::toFlat(*msg, projectuuiddummy);

      m_ioImage->writeImage(uuidstring, *imgMsg.GetRoot());
    }
    else
    {
      ROS_ERROR_STREAM("nullptr while iterating images");
    }
  }
}
void RosbagDumper::iterateAndDumpDetections()
{
  for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topicDetection)))
  {
    vision_msgs::Detection2DArray::ConstPtr msg = m.instantiate<vision_msgs::Detection2DArray>();
    if (msg != nullptr)
    {
      uint64_t time = (uint64_t)msg->header.stamp.sec << 32 | msg->header.stamp.nsec;
      std::string uuidstring;
      {  // scope of lock
        const std::scoped_lock lock(timeUuidMapMutex_);
        auto result = timeUuidMap_.find(time);
        if (result != timeUuidMap_.end())
        {
          uuidstring = result->second;
        }
      }
      ROS_INFO_STREAM("storing detections: " << uuidstring << " time: " << msg->header.stamp.sec << " / "
                                             << msg->header.stamp.nsec);

      if (uuidstring.empty())
      {
        ROS_ERROR_STREAM("uuid of detection is empty");
      }
      else
      {
        std::string projectuuiddummy = "";
        // extract labels
        std::vector<std::string> detections, instanceUUIDs;
        for (auto detection : msg->detections)
        {
          std::string label;
          switch (detection.results.at(0).id)
          {
            case 0:
              label = "MarkerYellow";
              break;
            case 1:
              label = "MarkerGreen";
              break;
            case 2:
              label = "Background";
              break;
            default:
              label = "error";
          }
          detections.push_back(label);
          std::string instanceUUID = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
          instanceUUIDs.push_back(instanceUUID);

          m_ioPoint->writePoint(boost::lexical_cast<std::string>(boost::uuids::random_generator()()),
                                createPointForDetection(detection, msg->header.stamp.sec, msg->header.stamp.nsec,
                                                        msg->header.frame_id, label, instanceUUID)
                                    .GetRoot());
        }

        m_ioImage->writeImageBoundingBox2DLabeled(
            uuidstring, seerep_ros_conversions_fb::toFlat(*msg, projectuuiddummy, detectionCategory, std::string(""),
                                                          detections, instanceUUIDs)
                            .GetRoot()
                            ->labels_bb());
      }
    }
    else
    {
      ROS_ERROR_STREAM("nullptr while iterating detections");
    }
  }
}
void RosbagDumper::iterateAndDumpTf()
{
  iterateAndDumpTf(topicTf, false);
  iterateAndDumpTf(topicTfStatic, true);
}
void RosbagDumper::iterateAndDumpTf(const std::string topic, const bool isStatic)
{
  rosbag::View view(bag, rosbag::TopicQuery(topic));

  for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(topic)))
  {
    tf2_msgs::TFMessage::ConstPtr msg = m.instantiate<tf2_msgs::TFMessage>();
    if (msg != nullptr)
    {
      for (auto tf : msg->transforms)
      {
        ROS_INFO_STREAM("storing tf " << tf.header.frame_id << " / " << tf.child_frame_id
                                      << "time: " << tf.header.stamp.sec << " / " << tf.header.stamp.nsec);

        std::string projectuuiddummy = "";
        auto tfMsg = seerep_ros_conversions_fb::toFlat(tf, projectuuiddummy, isStatic);
        m_ioTf->writeTransformStamped(*tfMsg.GetRoot());
      }
    }
    else
    {
      ROS_ERROR_STREAM("nullptr while iterating tf");
    }
  }
}

flatbuffers::grpc::Message<seerep::fb::PointStamped>
RosbagDumper::createPointForDetection(vision_msgs::Detection2D detection, int32_t stampSecs, uint32_t stampNanos,
                                      std::string frameId, std::string label, std::string instanceUUID)
{
  std::string projectuuiddummy = "";
  flatbuffers::grpc::MessageBuilder builder;
  auto header = seerep::fb::CreateHeader(
      builder, 0, seerep::fb::CreateTimestamp(builder, stampSecs, stampNanos), builder.CreateString(frameId),
      builder.CreateString(projectuuiddummy),
      builder.CreateString(boost::lexical_cast<std::string>(boost::uuids::random_generator()())));

  float x, y, z;  // project here
  projectPixel(detection.bbox.center.x, detection.bbox.center.y, distanceCameraGround, x, y, z);

  auto point = seerep::fb::CreatePoint(builder, x, y, z);

  std::vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>> labelVector;
  labelVector.push_back(seerep::fb::CreateLabelWithInstance(
      builder, seerep::fb::CreateLabel(builder, builder.CreateString(label), 1.0), builder.CreateString(instanceUUID)));

  std::vector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>> labelsWithInstanceWithCategoryVector;
  labelsWithInstanceWithCategoryVector.push_back(seerep::fb::CreateLabelsWithInstanceWithCategory(
      builder, builder.CreateString(detectionCategory), builder.CreateVector(labelVector)));

  std::vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>> attributes;
  attributes.push_back(
      seerep::fb::CreateUnionMapEntry(builder, builder.CreateString("plant_diameter"), seerep::fb::Datatypes_Double,
                                      seerep::fb::CreateDouble(builder, calcDiameter(detection)).Union()));

  auto pointStamped =
      seerep::fb::CreatePointStamped(builder, header, point, builder.CreateVector(labelsWithInstanceWithCategoryVector),
                                     builder.CreateVector(attributes));

  builder.Finish(pointStamped);

  return builder.ReleaseMessage<seerep::fb::PointStamped>();
}

void RosbagDumper::projectPixel(const float u, const float v, const float d, float& X, float& Y, float& Z)
{
  X = (u - cameraInfo.K.at(2)) * d / cameraInfo.K.at(0);
  Y = (v - cameraInfo.K.at(5)) * d / cameraInfo.K.at(4);
  Z = d;
}

float RosbagDumper::calcDiameter(vision_msgs::Detection2D detection)
{
  float x1, y1, z1;
  projectPixel(detection.bbox.center.x + detection.bbox.size_x, detection.bbox.center.y + detection.bbox.size_y,
               distanceCameraGround, x1, y1, z1);
  float x2, y2, z2;
  projectPixel(detection.bbox.center.x - detection.bbox.size_x, detection.bbox.center.y - detection.bbox.size_y,
               distanceCameraGround, x2, y2, z2);

  return std::max(std::abs(x1 - x2), std::abs(y1 - y2));
}

}  // namespace seerep_grpc_ros

std::string getHDF5FilePath(ros::NodeHandle private_nh)
{
  std::string hdf5FolderPath;
  if (!private_nh.getParam("hdf5FolderPath", hdf5FolderPath))
  {
    ROS_WARN_STREAM("Use the \"hdf5FolderPath\" parameter to specify the HDF5 file!");
    return "";
  }

  std::string projectUuid;
  if (private_nh.getParam("projectUuid", projectUuid))
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
      std::cout << e.what() << std::endl;
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
  ros::init(argc, argv, "seerep_ros_communication_rosbagDumper");
  ros::NodeHandle private_nh("~");

  std::string bagPath, hdf5FilePath, project_frame_id, project_name, topic_image, topic_camera_intrinsics,
      topic_detection, detection_category, topic_tf, topic_tf_static, topic_geo_anchor;
  float distance_camera_ground;

  hdf5FilePath = getHDF5FilePath(private_nh);

  if (private_nh.getParam("bag_path", bagPath) && private_nh.getParam("project_frame_id", project_frame_id) &&
      private_nh.getParam("project_name", project_name) && private_nh.getParam("topic_image", topic_image) &&
      private_nh.getParam("topic_camera_intrinsics", topic_camera_intrinsics) &&
      private_nh.getParam("topic_detection", topic_detection) &&
      private_nh.getParam("detection_category", detection_category) && private_nh.getParam("topic_tf", topic_tf) &&
      private_nh.getParam("topic_tf_static", topic_tf_static) &&
      private_nh.getParam("topic_geo_anchor", topic_geo_anchor) &&
      private_nh.param<float>("distance_camera_ground", distance_camera_ground, 0.0))
  {
    ROS_INFO_STREAM("hdf5FilePath: " << hdf5FilePath);
    ROS_INFO_STREAM("bagPath: " << bagPath);
    ROS_INFO_STREAM("project_frame_id: " << project_frame_id);
    ROS_INFO_STREAM("project_name: " << project_name);
    ROS_INFO_STREAM("topic_image: " << topic_image);
    ROS_INFO_STREAM("topic_camera_intrinsics: " << topic_camera_intrinsics);
    ROS_INFO_STREAM("topic_detection: " << topic_detection);
    ROS_INFO_STREAM("detection_category: " << detection_category);
    ROS_INFO_STREAM("topic_tf: " << topic_tf);
    ROS_INFO_STREAM("topic_tf_static: " << topic_tf_static);
    ROS_INFO_STREAM("topic_geo_anchor: " << topic_geo_anchor);
    ROS_INFO_STREAM("distance_camera_ground: " << distance_camera_ground);

    seerep_grpc_ros::RosbagDumper rosbagDumper(bagPath, hdf5FilePath, project_frame_id, project_name, topic_image,
                                               topic_camera_intrinsics, topic_detection, detection_category, topic_tf,
                                               topic_tf_static, topic_geo_anchor, distance_camera_ground);
  }
  else
  {
    ROS_ERROR_STREAM("Not all mandatory parameters are set!");
  }

  return EXIT_SUCCESS;
}
