#include "seerep_ros_communication/rosbagDumper.h"

namespace seerep_grpc_ros
{
RosbagDumper::RosbagDumper(const std::string& bagPath, const std::string& classesMappingPath,
                           const std::string& hdf5FilePath, const std::string& projectFrameId,
                           const std::string& projectName, const std::string& topicImage,
                           const std::string& topicCameraIntrinsics, const std::string& topicDetection,
                           const std::string& detectionCategory, const std::string& topicTf,
                           const std::string& topicTfStatic, const std::string& topicGeoAnchor,
                           float distanceCameraGround, bool storeImages)
  : hdf5FilePath(hdf5FilePath)
  , projectFrameId(projectFrameId)
  , projectName(projectName)
  , topicImage(topicImage)
  , topicCameraIntrinsics(topicCameraIntrinsics)
  , topicDetection(topicDetection)
  , detectionCategory(detectionCategory)
  , topicTf(topicTf)
  , topicTfStatic(topicTfStatic)
  , topicGeoAnchor(topicGeoAnchor)
  , distanceCameraGround(distanceCameraGround)
{
  auto writeMtx = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> hdf5File =
      std::make_shared<HighFive::File>(hdf5FilePath, HighFive::File::OpenOrCreate);
  ioCoreGeneral = std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(hdf5File, writeMtx);
  ioTf = std::make_shared<seerep_hdf5_fb::Hdf5FbTf>(hdf5File, writeMtx);
  ioImage = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(hdf5File, writeMtx);
  ioImageCore = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(hdf5File, writeMtx);
  ioPoint = std::make_shared<seerep_hdf5_fb::Hdf5FbPoint>(hdf5File, writeMtx);

  ioCoreGeneral->writeProjectname(std::filesystem::path(bagPath).filename());
  ioCoreGeneral->writeProjectFrameId("map");

  readClassesMapping(classesMappingPath);

  bag.open(bagPath);

  getGeoAnchor();
  getCameraIntrinsic();
  iterateAndDumpTf();
  if (storeImages)
  {
    iterateAndDumpImages();
  }
  iterateAndDumpDetections(storeImages);
}

RosbagDumper::~RosbagDumper()
{
  bag.close();
}

void RosbagDumper::readClassesMapping(const std::string& classesMappingPath)
{
  Json::Value classes;
  std::ifstream classesFile(classesMappingPath, std::ifstream::binary);
  classesFile >> classes;
  for (auto& root : classes)
  {
    for (Json::Value::const_iterator itr = root.begin(); itr != root.end(); itr++)
    {
      ROS_INFO_STREAM(std::stoi(itr.key().asCString()) << " ; " << itr->asCString());
      classesMapping.emplace(std::stoi(itr.key().asCString()), itr->asCString());
    }
  }
}

void RosbagDumper::getGeoAnchor()
{
  for (const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topicGeoAnchor)))
  {
    geographic_msgs::GeoPointStamped::ConstPtr msg = m.instantiate<geographic_msgs::GeoPointStamped>();
    if (msg != nullptr)
    {
      seerep_core_msgs::GeodeticCoordinates geoCoordinates;
      geoCoordinates.latitude = msg->position.latitude;
      geoCoordinates.longitude = msg->position.longitude;
      geoCoordinates.altitude = msg->position.altitude;
      geoCoordinates.ellipsoid = "";
      geoCoordinates.coordinateSystem = "";
      ioCoreGeneral->writeGeodeticLocation(geoCoordinates);
    }
  }
}

void RosbagDumper::getCameraIntrinsic()
{
  for (const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topicCameraIntrinsics)))
  {
    sensor_msgs::CameraInfo::ConstPtr msg = m.instantiate<sensor_msgs::CameraInfo>();
    if (msg != nullptr)
    {
      cameraInfo = *msg;
      break;
    }
  }
}

void RosbagDumper::iterateAndDumpImages()
{
  for (const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topicImage)))
  {
    sensor_msgs::Image::ConstPtr msg = m.instantiate<sensor_msgs::Image>();
    if (msg != nullptr)
    {
      boost::uuids::uuid msguuid = boost::uuids::random_generator()();
      std::string uuidstring = boost::lexical_cast<std::string>(msguuid);
      ROS_DEBUG_STREAM(uuidstring << " storing image"
                                  << " time: " << msg->header.stamp.sec << " / " << msg->header.stamp.nsec);
      uint64_t time = (uint64_t)msg->header.stamp.sec << 32 | msg->header.stamp.nsec;

      timeUuidMap.emplace(time, uuidstring);

      std::string projectuuiddummy = "";
      auto imgMsg = seerep_ros_conversions_fb::toFlat(*msg, projectuuiddummy);

      ioImage->writeImage(uuidstring, *imgMsg.GetRoot());
    }
    else
    {
      ROS_ERROR_STREAM("nullptr while iterating images");
    }
  }
}

void RosbagDumper::iterateAndDumpDetections(bool storeImages)
{
  for (const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topicDetection)))
  {
    vision_msgs::Detection2DArray::ConstPtr msg = m.instantiate<vision_msgs::Detection2DArray>();
    if (msg != nullptr)
    {
      uint64_t time = (uint64_t)msg->header.stamp.sec << 32 | msg->header.stamp.nsec;
      std::string uuidstring;
      if (storeImages)
      {
        auto result = timeUuidMap.find(time);
        if (result != timeUuidMap.end())
        {
          uuidstring = result->second;
        }

        ROS_INFO_STREAM("storing detections: " << uuidstring << " time: " << msg->header.stamp.sec << " / "
                                               << msg->header.stamp.nsec);
      }
      else
      {
        // generate a new uuid if image are not stored
        uuidstring = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      }

      if (uuidstring.empty())
      {
        ROS_ERROR_STREAM("uuid of detection is empty");
      }
      else
      {
        std::string projectuuiddummy = "";
        // extract labels
        std::vector<std::string> detections, instanceUUIDs;
        for (auto& detection : msg->detections)
        {
          std::string label = classesMapping.at(detection.results.at(0).id);
          ROS_INFO_STREAM("the label: " << label);
          detections.push_back(label);
          std::string instanceUUID = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
          instanceUUIDs.push_back(instanceUUID);

          ioPoint->writePoint(boost::lexical_cast<std::string>(boost::uuids::random_generator()()),
                              createPointForDetection(detection, msg->header.stamp.sec, msg->header.stamp.nsec,
                                                      msg->header.frame_id, label, instanceUUID)
                                  .GetRoot());
        }

        if (storeImages)
        {
          ioImage->writeImageBoundingBox2DLabeled(
              uuidstring, seerep_ros_conversions_fb::toFlat(*msg, projectuuiddummy, detectionCategory, std::string(""),
                                                            detections, instanceUUIDs)
                              .GetRoot()
                              ->labels_bb());
        }
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

void RosbagDumper::iterateAndDumpTf(const std::string& topic, const bool isStatic)
{
  rosbag::View view(bag, rosbag::TopicQuery(topic));

  for (const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topic)))
  {
    tf2_msgs::TFMessage::ConstPtr msg = m.instantiate<tf2_msgs::TFMessage>();
    if (msg != nullptr)
    {
      for (auto& tf : msg->transforms)
      {
        ROS_INFO_STREAM("storing tf " << tf.header.frame_id << " / " << tf.child_frame_id
                                      << "time: " << tf.header.stamp.sec << " / " << tf.header.stamp.nsec);

        std::string projectuuiddummy = "";
        auto tfMsg = seerep_ros_conversions_fb::toFlat(tf, projectuuiddummy, isStatic);
        ioTf->writeTransformStamped(*tfMsg.GetRoot());
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
                                      const std::string& frameId, const std::string& label,
                                      const std::string& instanceUUID)
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

std::string getHDF5FilePath(ros::NodeHandle privateNh)
{
  std::string hdf5FolderPath;
  if (!privateNh.getParam("hdf5FolderPath", hdf5FolderPath))
  {
    ROS_WARN_STREAM("Use the \"hdf5FolderPath\" parameter to specify the HDF5 file!");
    return "";
  }

  std::string projectUuid;
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
  ros::init(argc, argv, "seerep_ros_communication_rosbagDumper");
  ros::NodeHandle privateNh("~");

  std::string bagPath, classesMappingPath, projectFrameId, projectName, topicImage, topicCameraIntrinsics,
      topicDetection, detectionCategory, topicTf, topicTfStatic, topicGeoAnchor;
  float distanceCameraGround;
  bool storeImages;

  const std::string hdf5FilePath = getHDF5FilePath(privateNh);

  if (privateNh.getParam("bagPath", bagPath) && privateNh.getParam("classesMappingPath", classesMappingPath) &&
      privateNh.getParam("projectFrameId", projectFrameId) && privateNh.getParam("projectName", projectName) &&
      privateNh.getParam("topicImage", topicImage) &&
      privateNh.getParam("topicCameraIntrinsics", topicCameraIntrinsics) &&
      privateNh.getParam("topicDetection", topicDetection) &&
      privateNh.getParam("detectionCategory", detectionCategory) && privateNh.getParam("topicTf", topicTf) &&
      privateNh.getParam("topicTfStatic", topicTfStatic) && privateNh.getParam("topicGeoAnchor", topicGeoAnchor) &&
      privateNh.param<float>("distanceCameraGround", distanceCameraGround, 0.0) &&
      privateNh.param<bool>("storeImages", storeImages, true))
  {
    ROS_INFO_STREAM("hdf5FilePath: " << hdf5FilePath);
    ROS_INFO_STREAM("bagPath: " << bagPath);
    ROS_INFO_STREAM("classesMappingPath: " << classesMappingPath);
    ROS_INFO_STREAM("projectFrameId: " << projectFrameId);
    ROS_INFO_STREAM("projectName: " << projectName);
    ROS_INFO_STREAM("topicImage: " << topicImage);
    ROS_INFO_STREAM("topicCameraIntrinsics: " << topicCameraIntrinsics);
    ROS_INFO_STREAM("topicDetection: " << topicDetection);
    ROS_INFO_STREAM("detectionCategory: " << detectionCategory);
    ROS_INFO_STREAM("topicTf: " << topicTf);
    ROS_INFO_STREAM("topicTfStatic: " << topicTfStatic);
    ROS_INFO_STREAM("topicGeoAnchor: " << topicGeoAnchor);
    ROS_INFO_STREAM("distanceCameraGround: " << distanceCameraGround);
    ROS_INFO_STREAM("storeImages: " << storeImages);

    seerep_grpc_ros::RosbagDumper rosbagDumper(bagPath, classesMappingPath, hdf5FilePath, projectFrameId, projectName,
                                               topicImage, topicCameraIntrinsics, topicDetection, detectionCategory,
                                               topicTf, topicTfStatic, topicGeoAnchor, distanceCameraGround,
                                               storeImages);
  }
  else
  {
    ROS_ERROR_STREAM("Not all mandatory parameters are set!");
  }

  return EXIT_SUCCESS;
}
