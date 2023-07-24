#include "seerep_ros_communication/cognitive_weeding_rosbag_dumper.h"

namespace seerep_grpc_ros
{
RosbagDumper::RosbagDumper(const std::string& bagPath, const std::string& classesMappingPath,
                           const std::string& hdf5FilePath, const std::string& projectFrameId,
                           const std::string& projectName, const std::string& projectUuid,
                           const std::string& topicImage, const std::string& topicCameraIntrinsics,
                           const std::string& topicDetection, const std::string& detectionCategory,
                           const std::string& topicTf, const std::string& topicTfStatic,
                           const std::string& topicGeoAnchor, float distanceCameraGround, double maxViewingDistance,
                           bool storeImages)
  : hdf5FilePath(hdf5FilePath)
  , projectFrameId(projectFrameId)
  , projectName(projectName)
  , projectUuid(projectUuid)
  , topicImage(topicImage)
  , topicCameraIntrinsics(topicCameraIntrinsics)
  , topicDetection(topicDetection)
  , detectionCategory(detectionCategory)
  , topicTf(topicTf)
  , topicTfStatic(topicTfStatic)
  , topicGeoAnchor(topicGeoAnchor)
  , distanceCameraGround(distanceCameraGround)
  , maxViewingDistance(maxViewingDistance)
{
  auto writeMtx = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> hdf5File =
      std::make_shared<HighFive::File>(hdf5FilePath, HighFive::File::OpenOrCreate);
  ioCoreGeneral = std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(hdf5File, writeMtx);
  ioTf = std::make_shared<seerep_hdf5_fb::Hdf5FbTf>(hdf5File, writeMtx);
  ioImage = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(hdf5File, writeMtx);
  ioImageCore = std::make_shared<seerep_hdf5_core::Hdf5CoreImage>(hdf5File, writeMtx);
  ioPoint = std::make_shared<seerep_hdf5_fb::Hdf5FbPoint>(hdf5File, writeMtx);
  ioCameraIntrinsics = std::make_shared<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>(hdf5File, writeMtx);

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
      boost::uuids::uuid ciUuid = boost::uuids::random_generator()();
      cameraIntrinsicsUuid = boost::lexical_cast<std::string>(ciUuid);

      auto ci = seerep_core_fb::CoreFbConversion::fromFb(
          *seerep_ros_conversions_fb::toFlat(cameraInfo, projectUuid, cameraIntrinsicsUuid, maxViewingDistance)
               .GetRoot());
      ioCameraIntrinsics->writeCameraIntrinsics(ci);
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

      auto imgMsg = seerep_ros_conversions_fb::toFlat(*msg, projectUuid, cameraIntrinsicsUuid);

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
        // extract labels
        std::vector<std::string> detections, instanceUUIDs;
        for (auto& detection : msg->detections)
        {
          std::string label = classesMapping.at(detection.results.at(0).id);
          ROS_INFO_STREAM("the label: " << label);

          std::string concept = translateNameToAgrovocConcept(label);
          ROS_INFO_STREAM("the concept: " << concept);

          detections.push_back(concept);

          std::string instanceUUID = boost::lexical_cast<std::string>(boost::uuids::random_generator()());
          instanceUUIDs.push_back(instanceUUID);

          ioPoint->writePoint(boost::lexical_cast<std::string>(boost::uuids::random_generator()()),
                              createPointForDetection(detection, msg->header.stamp.sec, msg->header.stamp.nsec,
                                                      msg->header.frame_id, concept, instanceUUID)
                                  .GetRoot());
        }

        if (storeImages)
        {
          ioImage->writeImageBoundingBox2DLabeled(
              uuidstring, seerep_ros_conversions_fb::toFlat(*msg, projectUuid, detectionCategory, std::string(""),
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

        auto tfMsg = seerep_ros_conversions_fb::toFlat(tf, projectUuid, isStatic);
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
  flatbuffers::grpc::MessageBuilder builder;
  auto header = seerep::fb::CreateHeader(
      builder, 0, seerep::fb::CreateTimestamp(builder, stampSecs, stampNanos), builder.CreateString(frameId),
      builder.CreateString(projectUuid),
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

size_t writeCallback(void* contents, size_t size, size_t nmemb, std::string* output)
{
  size_t totalSize = size * nmemb;
  output->append((char*)contents, totalSize);
  return totalSize;
}

std::string RosbagDumper::translateNameToAgrovocConcept(std::string name)
{
  std::string concept = name;

  auto conceptCached = name2Concept.find(name);
  if (conceptCached != name2Concept.end())
  {
    concept = conceptCached->second;
  }
  else
  {
    CURL* curl = curl_easy_init();

    if (curl)
    {
      std::string sparqlQuery = std::string("PREFIX so: <http://schema.org/>\nPREFIX skos: "
                                            "<http://www.w3.org/2004/02/skos/core#>\n\nSELECT ?c ?l\nWHERE "
                                            "{\n  {\n    ?c skos:prefLabel \"") +
                                name +
                                std::string("\"@en.\n  }\n  UNION {\n    ?c skos:altLabel ?l FILTER (str(?l) = \"") +
                                name + std::string("\").\n  }\n}");

      // Set the Fuseki server URL and the dataset name
      std::string fusekiURL = "http://agrigaia-ur.ni.dfki:3030/plants/query";

      // Set the request parameters
      std::string postData =
          std::string("query=").append(curl_easy_escape(curl, sparqlQuery.c_str(), sparqlQuery.length()));

      // Set the HTTP POST headers
      struct curl_slist* headers = nullptr;
      headers = curl_slist_append(headers, "Accept: application/sparql-results+json");

      // Set the request options
      curl_easy_setopt(curl, CURLOPT_URL, fusekiURL.c_str());
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);

      // Response string to store the query result
      std::string response;
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

      // Perform the request
      CURLcode res = curl_easy_perform(curl);

      // Check for errors
      if (res != CURLE_OK)
      {
        std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
      }
      else
      {
        Json::Value root;
        Json::Reader reader;
        bool parsingSuccessful = reader.parse(response.c_str(), root);

        if (parsingSuccessful)
        {
          concept = root.get("results", name).get("bindings", name)[0].get("c", name).get("value", name).asString();
          name2Concept.emplace(name, concept);
        }
      }

      // Clean up
      curl_easy_cleanup(curl);
      curl_slist_free_all(headers);
    }
  }
  return concept;
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
  ros::init(argc, argv, "seerep_ros_communication_rosbagDumper");
  ros::NodeHandle privateNh("~");

  std::string bagPath, classesMappingPath, projectFrameId, projectName, topicImage, topicCameraIntrinsics,
      topicDetection, detectionCategory, topicTf, topicTfStatic, topicGeoAnchor;
  float distanceCameraGround;
  bool storeImages;
  double maxViewingDistance;

  std::string projectUuid;
  const std::string hdf5FilePath = getHDF5FilePath(privateNh, projectUuid);

  privateNh.getParam("classesMappingPath", classesMappingPath);
  privateNh.getParam("detectionCategory", detectionCategory);

  if (privateNh.getParam("bagPath", bagPath) && privateNh.getParam("classesMappingPath", classesMappingPath) &&
      privateNh.getParam("projectFrameId", projectFrameId) && privateNh.getParam("projectName", projectName) &&
      privateNh.getParam("topicImage", topicImage) &&
      privateNh.getParam("topicCameraIntrinsics", topicCameraIntrinsics) &&
      privateNh.getParam("topicDetection", topicDetection) && privateNh.getParam("topicTf", topicTf) &&
      privateNh.getParam("topicTfStatic", topicTfStatic) && privateNh.getParam("topicGeoAnchor", topicGeoAnchor) &&
      privateNh.param<float>("distanceCameraGround", distanceCameraGround, 0.0) &&
      privateNh.param<double>("maxViewingDistance", maxViewingDistance, 0.0) &&
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
    ROS_INFO_STREAM("maxViewingDistance: " << maxViewingDistance);
    ROS_INFO_STREAM("storeImages: " << storeImages);

    seerep_grpc_ros::RosbagDumper rosbagDumper(bagPath, classesMappingPath, hdf5FilePath, projectFrameId, projectName,
                                               projectUuid, topicImage, topicCameraIntrinsics, topicDetection,
                                               detectionCategory, topicTf, topicTfStatic, topicGeoAnchor,
                                               distanceCameraGround, maxViewingDistance, storeImages);
  }
  else
  {
    ROS_ERROR_STREAM("Not all mandatory parameters are set!");
  }

  return EXIT_SUCCESS;
}
