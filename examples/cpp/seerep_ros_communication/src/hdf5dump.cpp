#include "seerep_ros_communication/hdf5dump.h"

namespace seerep_grpc_ros
{
  DumpSensorMsgs::DumpSensorMsgs(std::string hdf5FilePath, std::string project_frame_id, std::string project_name, std::string projectUuid)
  {
    ROS_INFO_STREAM("creating a new project with name: \"" << project_name << "\" and frame id: \"" << project_frame_id
                                                           << "\" in  file: \"" << hdf5FilePath << "\"");
    std::vector<std::string> labelsAsStdVector, instancesAsStdVector;
    std::vector<float> confidenceAsStdVector;
    labelsAsStdVector.push_back("sun");
    confidenceAsStdVector.push_back(1.0);
    // no instances, just labels -> no uuids
    instancesAsStdVector.push_back("");

    seerep_core_msgs::LabelsWithInstanceWithCategory labelsWithInstanceWithCategory;
    labelsWithInstanceWithCategory.category = "weather";
    labelsWithInstanceWithCategory.instances = instancesAsStdVector;
    labelsWithInstanceWithCategory.labels = labelsAsStdVector;
    labelsWithInstanceWithCategory.labelConfidences = confidenceAsStdVector;

    m_labelsWithInstanceWithCategory.push_back(labelsWithInstanceWithCategory);


    // camera instrinsic uuid
    boost::uuids::uuid ciUuid = boost::uuids::random_generator()();
    m_cameraIntrinsicsUuid = boost::lexical_cast<std::string>(ciUuid);
    // max viewing distance for uuid
    m_maxViewingDistance = 15.0;
    //project uuid
    m_projectUuid = projectUuid;

    auto write_mtx = std::make_shared<std::mutex>();
    std::shared_ptr<HighFive::File> hdf5_file =
        std::make_shared<HighFive::File>(hdf5FilePath, HighFive::File::OpenOrCreate);
    auto ioGeneral = seerep_hdf5_core::Hdf5CoreGeneral(hdf5_file, write_mtx);
    ioGeneral.writeProjectFrameId(project_frame_id);
    ioGeneral.writeProjectname(project_name);

    m_ioTf = std::make_shared<seerep_hdf5_pb::Hdf5PbTf>(hdf5_file, write_mtx);
    m_ioPointCloud = std::make_shared<seerep_hdf5_fb::Hdf5FbPointCloud>(hdf5_file, write_mtx);
    m_ioImage = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(hdf5_file, write_mtx);
    m_ioImageCore = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(hdf5_file, write_mtx);
    m_ioCameraIntrinsic = std::make_shared<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>(hdf5_file, write_mtx);

  }

  void DumpSensorMsgs::dump(const std_msgs::Header::ConstPtr &msg) const
  {
    (void)msg; // ignore that variable without causing warnings
    ROS_INFO_STREAM("Datatype not implemented.");
  }

  void DumpSensorMsgs::dump(const sensor_msgs::PointCloud2::ConstPtr &msg) const
  {
    try
    {
      std::string uuid = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      ROS_INFO_STREAM("Dump point cloud 2 with uuid: " << uuid);
      /*
     TODO: Temporary workaround because the Protobuf HDF5 PCL storage produces the wrong '\points' dataset layout,
     due to the changes introduced in PR #354. It uses an NxM dimensional float dataset instead of an 1x(N*M) byte
     dataset. This workaround should be removed as soon as the Protobuf PCL storage is ported to the new layout.
      */
      auto pcl = seerep_ros_conversions_fb::toFlat(*msg, uuid);
      // TODO: ... ::toFlat(*msg, uuid).GetRoot() causes a segfault when accessing the data field of the pcl?!?
      auto [min_corner, max_corner] = m_ioPointCloud->computeBoundingBox(*pcl.GetRoot());
      m_ioPointCloud->writeBoundingBox(uuid, min_corner, max_corner);
      m_ioPointCloud->writePointCloud2(uuid, *pcl.GetRoot());
      m_ioImageCore->writeLabelsGeneral(uuid, m_labelsWithInstanceWithCategory);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Exception while saving point cloud: " << e.what());
    }
  }

  void DumpSensorMsgs::dump(const vision_msgs::Detection3D::ConstPtr &msg) const
  {
    try
    {
      const std::string uuid = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      ROS_INFO_STREAM("Dump Detection3D with uuid: " << uuid);

      /* store pcl and it's bounding box */
      auto pcl = seerep_ros_conversions_fb::toFlat(msg->source_cloud, uuid);
      m_ioPointCloud->writePointCloud2(uuid, *pcl.GetRoot());
      auto [min_corner, max_corner] = m_ioPointCloud->computeBoundingBox(*pcl.GetRoot());
      m_ioPointCloud->writeBoundingBox(uuid, min_corner, max_corner);

      flatbuffers::grpc::MessageBuilder fbb;

      const std::string instanceUUID = boost::lexical_cast<std::string>(boost::uuids::random_generator()());

      auto labelWithInstance = seerep::fb::CreateLabelWithInstance(
          fbb, seerep::fb::CreateLabel(fbb, fbb.CreateString("Person1"), 1.0), fbb.CreateString(instanceUUID));

      auto boundingBox =
          seerep::fb::CreateBoundingbox(fbb, seerep::fb::CreatePoint(fbb, 1, 1, 1), seerep::fb::CreatePoint(fbb, 10, 10, 10),
                                        seerep::fb::CreateQuaternion(fbb, 0, 0, 0, 1));

      auto boundingBoxLabeled = seerep::fb::CreateBoundingBoxLabeled(fbb, labelWithInstance, boundingBox);
      std::vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>> BoundingBoxesLabeled = {boundingBoxLabeled};

      auto boundingBoxLabeledWithCategory = seerep::fb::CreateBoundingBoxLabeledWithCategory(
          fbb, fbb.CreateString("AutoGeneratedGroundTruth"), fbb.CreateVector(BoundingBoxesLabeled));
      std::vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeledWithCategory>> boxes = {
          boundingBoxLabeledWithCategory};

      auto header = seerep_ros_conversions_fb::toFlat(msg->header, std::string(""), fbb, std::string(""));
      auto bbStamped = seerep::fb::CreateBoundingBoxesLabeledStamped(fbb, header, fbb.CreateVector(boxes));
      fbb.Finish(bbStamped);
      auto releasedMessage = fbb.ReleaseMessage<seerep::fb::BoundingBoxesLabeledStamped>();

      m_ioPointCloud->writePointCloudBoundingBoxLabeled(uuid, releasedMessage.GetRoot()->labels_bb());
      m_ioImageCore->writeLabelsGeneral(uuid, m_labelsWithInstanceWithCategory);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Exception while saving point cloud with detection: " << e.what());
    }
  }

  void DumpSensorMsgs::dump(const sensor_msgs::Image::ConstPtr &msg) const
  {
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    std::string uuidString = boost::lexical_cast<std::string>(uuid);
    boost::uuids::uuid ciUuid = boost::uuids::random_generator()();
    std::string cameraIntrinsicsUuid = boost::lexical_cast<std::string>(ciUuid);
    try
    {
      auto imgMsg = seerep_ros_conversions_fb::toFlat(*msg, m_projectUuid, cameraIntrinsicsUuid);
      m_ioImage->writeImage(uuidString, *imgMsg.GetRoot());
      m_ioImageCore->writeLabelsGeneral(uuidString, m_labelsWithInstanceWithCategory);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Exception while saving image: " << e.what());
    }
  }

  void DumpSensorMsgs::dump(const sensor_msgs::CameraInfo::ConstPtr &msg) const
  {
    try
    {
      auto ci = seerep_core_fb::CoreFbConversion::fromFb(
           *seerep_ros_conversions_fb::toFlat(*msg, const_cast<std::string&>(m_projectUuid), const_cast<std::string&>(m_cameraIntrinsicsUuid), m_maxViewingDistance)
                .GetRoot());
      m_ioCameraIntrinsic->writeCameraIntrinsics(ci);
    }
    catch (const std::exception &e)
    {
      ROS_ERROR_STREAM("Exception while saving camera info: " << e.what());
    }
  }

  void DumpSensorMsgs::dump(const vision_msgs::Detection2DArray::ConstPtr &msg) const
  {
    // extract labels
    ROS_INFO_STREAM("Num of Detecitons " << msg->detections.size() - 1);
    bool image_sent = false;
    std::vector<std::string> detections, instanceUUIDs;
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    std::string uuidString = boost::lexical_cast<std::string>(uuid);

    flatbuffers::grpc::MessageBuilder fbb;
    std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>> BoundingBoxes2DLabeled;
    for (auto &detection : msg->detections)
    {
      if (!image_sent)
      {
        try
        {
          auto imgMsg = seerep_ros_conversions_fb::toFlat(detection.source_img, m_projectUuid, m_cameraIntrinsicsUuid);
          m_ioImage->writeImage(uuidString, *imgMsg.GetRoot());
          // also write the labels general; filled with dummy data right now
          m_ioImageCore->writeLabelsGeneral(uuidString, m_labelsWithInstanceWithCategory);
          ROS_INFO_STREAM("Image sent: " << uuidString);
          image_sent = true;
        }
        catch (const std::exception &e)
        {
          ROS_ERROR_STREAM("Exception while saving image of detection: " << e.what());
        }
      }

      std::string label = "person"; // ToDo IDtoString(detection.results.at(0).id);

      detections.push_back(label);

      std::string instanceUUID = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      instanceUUIDs.push_back(instanceUUID);
      ROS_INFO_STREAM("Added label: " << label);

      auto boundingBox2DLabeled = seerep_ros_conversions_fb::toFlat(detection, fbb, label, instanceUUID);

      BoundingBoxes2DLabeled.push_back(boundingBox2DLabeled);

      ROS_INFO_STREAM("Added Bounding Box");
    }

    // write Bounding Boxes
    auto BoundingBox2DLabeledWithCategory = seerep::fb::CreateBoundingBox2DLabeledWithCategory(fbb, fbb.CreateString("AutoGeneratedGroundTruth"), fbb.CreateVector(BoundingBoxes2DLabeled));
    std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>> BoundingBoxes2DLabeledWithCategory = {BoundingBox2DLabeledWithCategory};
    auto header = seerep_ros_conversions_fb::toFlat(msg->header, std::string(""), fbb, std::string(""));
    auto bbStamped = seerep::fb::CreateBoundingBoxes2DLabeledStamped(fbb, header, fbb.CreateVector(BoundingBoxes2DLabeledWithCategory));
    fbb.Finish(bbStamped);
    auto releasedMessage = fbb.ReleaseMessage<seerep::fb::BoundingBoxes2DLabeledStamped>();

    m_ioImage->writeImageBoundingBox2DLabeled(uuidString, releasedMessage.GetRoot()->labels_bb());
    ROS_INFO_STREAM("Bounding Boxes added to " << uuidString);
  }

  void DumpSensorMsgs::dump(const geometry_msgs::Point::ConstPtr &msg) const
  {
    (void)msg; // ignore that variable without causing warnings
    ROS_INFO_STREAM("Datatype not implemented.");
  }

  void DumpSensorMsgs::dump(const geometry_msgs::Quaternion::ConstPtr &msg) const
  {
    (void)msg; // ignore that variable without causing warnings
    ROS_INFO_STREAM("Datatype not implemented.");
  }

  void DumpSensorMsgs::dump(const geometry_msgs::Pose::ConstPtr &msg) const
  {
    (void)msg; // ignore that variable without causing warnings
    ROS_INFO_STREAM("Datatype not implemented.");
  }

  void DumpSensorMsgs::dump(const geometry_msgs::PoseStamped::ConstPtr &msg) const
  {
    (void)msg; // ignore that variable without causing warnings
    ROS_INFO_STREAM("Datatype not implemented.");
  }

  void DumpSensorMsgs::dump(const tf2_msgs::TFMessage::ConstPtr &msg) const
  {
    for (geometry_msgs::TransformStamped transform : msg->transforms)
    {
      try
      {
        m_ioTf->writeTransformStamped(seerep_ros_conversions_pb::toProto(transform, false));
      }
      catch (const std::exception &e)
      {
        ROS_ERROR_STREAM("Exception while saving transformation: " << e.what());
      }
    }
  }

  std::optional<ros::Subscriber> DumpSensorMsgs::getSubscriber(const std::string &message_type, const std::string &topic)
  {
    switch (type(message_type))
    {
    case std_msgs_Header:
      return nh.subscribe<std_msgs::Header>(topic, 0, &DumpSensorMsgs::dump, this);
    case sensor_msgs_Image:
      return nh.subscribe<sensor_msgs::Image>(topic, 0, &DumpSensorMsgs::dump, this);
    case sensor_msgs_CameraInfo:
      return nh.subscribe<sensor_msgs::CameraInfo>(topic, 0, &DumpSensorMsgs::dump, this);
    case vision_msgs_Detection2DArray:
      return nh.subscribe<vision_msgs::Detection2DArray>(topic, 0, &DumpSensorMsgs::dump, this);
    case sensor_msgs_PointCloud2:
      return nh.subscribe<sensor_msgs::PointCloud2>(topic, 0, &DumpSensorMsgs::dump, this);
    case vision_msgs_Detection3D:
      return nh.subscribe<vision_msgs::Detection3D>(topic, 0, &DumpSensorMsgs::dump, this);
    case geometry_msgs_Point:
      return nh.subscribe<geometry_msgs::Point>(topic, 0, &DumpSensorMsgs::dump, this);
    case geometry_msgs_Quaternion:
      return nh.subscribe<geometry_msgs::Quaternion>(topic, 0, &DumpSensorMsgs::dump, this);
    case geometry_msgs_Pose:
      return nh.subscribe<geometry_msgs::Pose>(topic, 0, &DumpSensorMsgs::dump, this);
    case geometry_msgs_PoseStamped:
      return nh.subscribe<geometry_msgs::PoseStamped>(topic, 0, &DumpSensorMsgs::dump, this);
    case tf2_msgs_TFMessage:
      return nh.subscribe<tf2_msgs::TFMessage>(topic, 0, &DumpSensorMsgs::dump, this);
    default:
      ROS_ERROR_STREAM("Type \"" << message_type << "\" not supported");
      return std::nullopt;
    }
  }
} // namespace seerep_grpc_ros

int main(int argc, char **argv)
{
  ros::init(argc, argv, "seerep_ros_communication_hdf5_dump");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::map<std::string, ros::Subscriber> subscribers;
  std::vector<std::string> topics;

  std::string hdf5FolderPath;
  if (!private_nh.getParam("hdf5FolderPath", hdf5FolderPath))
  {
    ROS_WARN_STREAM("Use the \"hdf5FolderPath\" parameter to specify the HDF5 file!");
    return EXIT_FAILURE;
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
    catch (std::runtime_error const &e)
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

  std::string hdf5FilePath = hdf5FolderPath + "/" + projectUuid + ".h5";

  std::string project_frame_id, project_name;
  if (!private_nh.getParam("project_frame_id", project_frame_id))
  {
    ROS_WARN_STREAM("Use the \"project_frame_id\" parameter to specify the base frame id of the Seerep project! The "
                    "\"project_frame_id\" parameter should be a string.");
  }
  if (!private_nh.getParam("project_name", project_name))
  {
    ROS_WARN_STREAM("Use the \"project_name\" parameter to specify the name of the Seerep project! The "
                    "\"project_name\" parameter should be a string.");
  }

  seerep_grpc_ros::DumpSensorMsgs dumpSensorMsgs =
      seerep_grpc_ros::DumpSensorMsgs(hdf5FilePath, project_frame_id, project_name, projectUuid);

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  if (!private_nh.getParam("topics", topics))
  {
    ROS_WARN_STREAM("Use the \"topics\" parameter to specify the ROS topics which should be transferred! The "
                    "\"topics\" parameter should be a list of strings.");
  }

  ROS_INFO_STREAM("Type names: " << seerep_grpc_ros::names());

  for (auto topic : topics)
  {
    ROS_INFO_STREAM("Trying to subscribe to topic \"" << topic << "\".");
  }

  for (auto info : topic_info)
  {
    auto find_iter = std::find(topics.begin(), topics.end(), info.name);

    if (find_iter != topics.end())
    {
      auto sub_opt = dumpSensorMsgs.getSubscriber(info.datatype, info.name);
      if (sub_opt)
      {
        ROS_INFO_STREAM("Subscribe to topic: \"" << info.name << "\" of type:\"" << info.datatype << "\".");
        subscribers[info.name] = *sub_opt;
      }
      topics.erase(find_iter);
    }
    else
    {
      ROS_INFO_STREAM("Available Topics: \"" << info.name << "\"");
    }
  }

  ros::spin();

  return EXIT_SUCCESS;
}
