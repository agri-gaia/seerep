#include "seerep_ros_communication/jsonPointDumper.h"

namespace seerep_grpc_ros
{
JsonPointDumper::JsonPointDumper(const std::string& filePath, const std::string& hdf5FilePath,
                                 const std::string& detectionCategory)
  : detectionCategory(detectionCategory)
{
  auto writeMtx = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> hdf5File =
      std::make_shared<HighFive::File>(hdf5FilePath, HighFive::File::OpenOrCreate);
  ioCoreGeneral = std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(hdf5File, writeMtx);
  ioPoint = std::make_shared<seerep_hdf5_fb::Hdf5FbPoint>(hdf5File, writeMtx);

  ioCoreGeneral->writeProjectname(std::filesystem::path(filePath).filename());
  ioCoreGeneral->writeProjectFrameId("world");

  seerep_core_msgs::GeodeticCoordinates geoCoordinates;
  geoCoordinates.latitude = 0.0;
  geoCoordinates.longitude = 0.0;
  geoCoordinates.altitude = 0.0;
  geoCoordinates.ellipsoid = "WGS84";
  geoCoordinates.coordinateSystem = "EPSG:32632";
  ioCoreGeneral->writeGeodeticLocation(geoCoordinates);

  readAndDumpJson(filePath);
}

JsonPointDumper::~JsonPointDumper()
{
}

void JsonPointDumper::readAndDumpJson(const std::string& jsonFilePath)
{
  std::ifstream file(jsonFilePath, std::ifstream::binary);
  Json::Reader reader;
  // this will contain complete JSON data
  Json::Value completeJsonData;
  // reader reads the data and stores it in completeJsonData
  reader.parse(file, completeJsonData);
  auto features = completeJsonData["features"];
  std::cout << features[0] << std::endl;

  for (auto detection : completeJsonData["features"])
  {
    double x = (detection["properties"]["maxx"].asDouble() - detection["properties"]["minx"].asDouble()) / 2.0 +
               detection["properties"]["minx"].asDouble();
    double y = (detection["properties"]["maxy"].asDouble() - detection["properties"]["miny"].asDouble()) / 2.0 +
               detection["properties"]["miny"].asDouble();
    float z = 0.0;

    float diameter =
        std::max(std::abs(detection["properties"]["minx"].asDouble() - detection["properties"]["maxx"].asDouble()),
                 std::abs(detection["properties"]["miny"].asDouble() - detection["properties"]["maxy"].asDouble()));

    std::string label;
    if (detection["properties"]["botanical"].asString() == "-")
    {
      label = detection["properties"]["german"].asString();
    }
    else
    {
      label = detection["properties"]["botanical"].asString();
    }
    std::vector<std::string> detections;
    detections.push_back(label);
    std::string instanceUUID = boost::lexical_cast<std::string>(boost::uuids::random_generator()());

    ioPoint->writePoint(boost::lexical_cast<std::string>(boost::uuids::random_generator()()),
                        createPointForDetection(0, 0, "world", label, instanceUUID, x, y, z, diameter).GetRoot());
  }
}

flatbuffers::grpc::Message<seerep::fb::PointStamped>
JsonPointDumper::createPointForDetection(int32_t stampSecs, uint32_t stampNanos, const std::string& frameId,
                                         const std::string& label, const std::string& instanceUUID, const float x,
                                         const float y, const float z, const float diameter)
{
  std::string projectuuiddummy = "";
  flatbuffers::grpc::MessageBuilder builder;
  auto header = seerep::fb::CreateHeader(
      builder, 0, seerep::fb::CreateTimestamp(builder, stampSecs, stampNanos), builder.CreateString(frameId),
      builder.CreateString(projectuuiddummy),
      builder.CreateString(boost::lexical_cast<std::string>(boost::uuids::random_generator()())));

  auto point = seerep::fb::CreatePoint(builder, x, y, z);

  std::vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>> labelVector;
  labelVector.push_back(seerep::fb::CreateLabelWithInstance(
      builder, seerep::fb::CreateLabel(builder, builder.CreateString(label), 1.0), builder.CreateString(instanceUUID)));

  std::vector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>> labelsWithInstanceWithCategoryVector;
  labelsWithInstanceWithCategoryVector.push_back(seerep::fb::CreateLabelsWithInstanceWithCategory(
      builder, builder.CreateString(detectionCategory), builder.CreateVector(labelVector)));

  std::vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>> attributes;
  attributes.push_back(seerep::fb::CreateUnionMapEntry(builder, builder.CreateString("plant_diameter"),
                                                       seerep::fb::Datatypes_Double,
                                                       seerep::fb::CreateDouble(builder, diameter).Union()));

  auto pointStamped =
      seerep::fb::CreatePointStamped(builder, header, point, builder.CreateVector(labelsWithInstanceWithCategoryVector),
                                     builder.CreateVector(attributes));

  builder.Finish(pointStamped);

  return builder.ReleaseMessage<seerep::fb::PointStamped>();
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
  ros::init(argc, argv, "seerep_ros_communication_JsonPointDumper");
  ros::NodeHandle privateNh("~");

  std::string jsonFilePath, detectionCategory;

  const std::string hdf5FilePath = getHDF5FilePath(privateNh);

  if (privateNh.getParam("jsonFilePath", jsonFilePath) && privateNh.getParam("detectionCategory", detectionCategory))
  {
    {
      ROS_INFO_STREAM("hdf5FilePath: " << hdf5FilePath);
      ROS_INFO_STREAM("jsonFilePath: " << jsonFilePath);
      ROS_INFO_STREAM("detectionCategory: " << detectionCategory);

      seerep_grpc_ros::JsonPointDumper JsonPointDumper(jsonFilePath, hdf5FilePath, detectionCategory);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Not all mandatory parameters are set!");
  }

  return EXIT_SUCCESS;
}
