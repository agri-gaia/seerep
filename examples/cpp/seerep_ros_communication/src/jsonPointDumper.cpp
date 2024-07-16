#include "seerep_ros_communication/jsonPointDumper.h"

namespace seerep_grpc_ros
{
JsonPointDumper::JsonPointDumper(const std::string& filePath,
                                 const std::string& hdf5FilePath,
                                 const std::string& dataSource,
                                 const std::string& classesMappingPath)
{
  if (!std::filesystem::exists(filePath))
  {
    throw std::invalid_argument(filePath + " does not exist.");
  }
  if (!std::filesystem::exists(classesMappingPath))
  {
    throw std::invalid_argument(classesMappingPath + " does not exist.");
  }

  auto writeMtx = std::make_shared<std::mutex>();
  std::shared_ptr<HighFive::File> hdf5File = std::make_shared<HighFive::File>(
      hdf5FilePath, HighFive::File::OpenOrCreate);
  ioCoreGeneral =
      std::make_shared<seerep_hdf5_core::Hdf5CoreGeneral>(hdf5File, writeMtx);
  ioPoint = std::make_shared<seerep_hdf5_fb::Hdf5FbPoint>(hdf5File, writeMtx);

  ioCoreGeneral->writeProjectname(std::filesystem::path(filePath).filename());

  if (dataSource == "uos")
  {
    readAndDumpJsonUos(filePath);
  }
  else if (dataSource == "fr")
  {
    readAndDumpJsonFr(filePath, classesMappingPath);
  }
  else
  {
    ROS_ERROR_STREAM("no valid data source. Set to \"uos\" or \"fr\"");
  }
}

JsonPointDumper::~JsonPointDumper()
{
}

void JsonPointDumper::readAndDumpJsonUos(const std::string& jsonFilePath)
{
  const std::string frame = "world";
  ioCoreGeneral->writeProjectFrameId(frame);

  // uos data is in world coordinates; set to 0
  seerep_core_msgs::GeodeticCoordinates geoCoordinates;
  geoCoordinates.latitude = 0.0;
  geoCoordinates.longitude = 0.0;
  geoCoordinates.altitude = 0.0;
  geoCoordinates.coordinateSystem = "EPSG:32632";
  ioCoreGeneral->writeGeodeticLocation(geoCoordinates);

  std::ifstream file(jsonFilePath, std::ifstream::binary);
  Json::Reader reader;
  // this will contain complete JSON data
  Json::Value completeJsonData;
  // reader reads the data and stores it in completeJsonData
  reader.parse(file, completeJsonData);

  for (auto detection : completeJsonData["features"])
  {
    double x = (detection["properties"]["maxx"].asDouble() -
                detection["properties"]["minx"].asDouble()) /
                   2.0 +
               detection["properties"]["minx"].asDouble();
    double y = (detection["properties"]["maxy"].asDouble() -
                detection["properties"]["miny"].asDouble()) /
                   2.0 +
               detection["properties"]["miny"].asDouble();
    double z = 0.0;

    double diameter =
        std::max(std::abs(detection["properties"]["minx"].asDouble() -
                          detection["properties"]["maxx"].asDouble()),
                 std::abs(detection["properties"]["miny"].asDouble() -
                          detection["properties"]["maxy"].asDouble()));

    std::string trivialName;
    if (detection["properties"]["botanical"].asString() == "-")
    {
      trivialName = detection["properties"]["german"].asString();
    }
    else
    {
      trivialName = detection["properties"]["botanical"].asString();
    }
    std::string concept = translateNameToAgrovocConcept(trivialName);

    std::string instanceUUID =
        boost::lexical_cast<std::string>(boost::uuids::random_generator()());

    ioPoint->writePoint(
        boost::lexical_cast<std::string>(boost::uuids::random_generator()()),
        createPointForDetection(0, 0, frame, concept, trivialName, instanceUUID,
                                x, y, z, diameter)
            .GetRoot());
  }
}

void JsonPointDumper::readAndDumpJsonFr(const std::string& jsonFilePath,
                                        const std::string& classesMappingPath)
{
  const std::string frame = "map";
  ioCoreGeneral->writeProjectFrameId(frame);

  std::ifstream file(jsonFilePath, std::ifstream::binary);
  Json::Reader reader;
  // this will contain complete JSON data
  Json::Value completeJsonData;
  // reader reads the data and stores it in completeJsonData
  reader.parse(file, completeJsonData);

  seerep_core_msgs::GeodeticCoordinates geoCoordinates;
  geoCoordinates.latitude =
      completeJsonData["anchor_lla"]["latitude"].asDouble();
  geoCoordinates.longitude =
      completeJsonData["anchor_lla"]["longitude"].asDouble();
  geoCoordinates.altitude = 0.0;
  geoCoordinates.coordinateSystem = "EPSG:4326";
  ioCoreGeneral->writeGeodeticLocation(geoCoordinates);

  std::unordered_map<int64_t, std::string> classesMapping =
      readClassesMapping(classesMappingPath);

  for (auto detection : completeJsonData["plants"])
  {
    double x = detection["center_x"].asDouble();
    double y = detection["center_y"].asDouble();
    double z = 0.0;

    double diameter = detection["areas"][0].asDouble();

    std::string trivialName =
        classesMapping.at(detection["class_ids"][0].asInt());
    std::string concept = translateNameToAgrovocConcept(trivialName);
    double confidence = detection["confidences"][0].asDouble();

    std::string instanceUUID =
        boost::lexical_cast<std::string>(boost::uuids::random_generator()());

    ioPoint->writePoint(
        boost::lexical_cast<std::string>(boost::uuids::random_generator()()),
        createPointForDetection(0, 0, frame, concept, trivialName, instanceUUID,
                                x, y, z, diameter, confidence)
            .GetRoot());
  }
}

std::unordered_map<int64_t, std::string>
JsonPointDumper::readClassesMapping(const std::string& classesMappingPath)
{
  if (classesMappingPath == "")
  {
    throw(std::invalid_argument(
        "classesMappingPath must be set when using fr data source"));
  }
  std::unordered_map<int64_t, std::string> classesMapping;

  Json::Value classes;
  std::ifstream classesFile(classesMappingPath, std::ifstream::binary);
  classesFile >> classes;
  for (auto& root : classes)
  {
    for (Json::Value::const_iterator itr = root.begin(); itr != root.end();
         itr++)
    {
      ROS_INFO_STREAM(std::stoi(itr.key().asCString())
                      << " ; " << itr->asCString());
      classesMapping.emplace(std::stoi(itr.key().asCString()), itr->asCString());
    }
  }
  return classesMapping;
}

flatbuffers::grpc::Message<seerep::fb::PointStamped>
JsonPointDumper::createPointForDetection(
    int32_t stampSecs, uint32_t stampNanos, const std::string& frameId,
    const std::string& labelAgrovoc, const std::string& labelTrivial,
    const std::string& instanceUUID, const double x, const double y,
    const double z, const double diameter, const double confidence)
{
  std::string projectuuiddummy = "";
  flatbuffers::grpc::MessageBuilder builder;
  auto header = seerep::fb::CreateHeader(
      builder, 0, seerep::fb::CreateTimestamp(builder, stampSecs, stampNanos),
      builder.CreateString(frameId), builder.CreateString(projectuuiddummy),
      builder.CreateString(boost::lexical_cast<std::string>(
          boost::uuids::random_generator()())));

  auto point = seerep::fb::CreatePoint(builder, x, y, z);

  std::vector<flatbuffers::Offset<seerep::fb::Label>> labelAgrovocVector;
  labelAgrovocVector.push_back(
      seerep::fb::CreateLabel(builder, builder.CreateString(labelAgrovoc), -1,
                              builder.CreateString(instanceUUID), -1));

  std::vector<flatbuffers::Offset<seerep::fb::Label>> labelTrivialVector;
  labelTrivialVector.push_back(
      seerep::fb::CreateLabel(builder, builder.CreateString(labelTrivial), -1,
                              builder.CreateString(instanceUUID), -1));

  // TODO create datumaro json based on the labels
  std::string datumaroJson;

  std::vector<flatbuffers::Offset<seerep::fb::LabelCategory>>
      labelsCategoryVector;
  labelsCategoryVector.push_back(
      seerep::fb::CreateLabelCategory(builder, builder.CreateString("agrovoc"),
                                      builder.CreateVector(labelAgrovocVector),
                                      builder.CreateString(datumaroJson)));
  labelsCategoryVector.push_back(
      seerep::fb::CreateLabelCategory(builder, builder.CreateString("trivial"),
                                      builder.CreateVector(labelTrivialVector),
                                      builder.CreateString(datumaroJson)));

  std::vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>> attributes;
  attributes.push_back(seerep::fb::CreateUnionMapEntry(
      builder, builder.CreateString("plant_diameter"),
      seerep::fb::Datatypes_Double,
      seerep::fb::CreateDouble(builder, diameter).Union()));

  auto pointStamped =
      seerep::fb::CreatePointStamped(builder, header, point,
                                     builder.CreateVector(labelsCategoryVector),
                                     builder.CreateVector(attributes));

  builder.Finish(pointStamped);

  return builder.ReleaseMessage<seerep::fb::PointStamped>();
}

size_t writeCallback(void* contents, size_t size, size_t nmemb,
                     std::string* output)
{
  size_t totalSize = size * nmemb;
  output->append((char*)contents, totalSize);
  return totalSize;
}

std::string JsonPointDumper::translateNameToAgrovocConcept(std::string name)
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
      std::string sparqlQuery =
          std::string(
              "PREFIX so: <http://schema.org/>\nPREFIX skos: "
              "<http://www.w3.org/2004/02/skos/core#>\n\nSELECT ?c ?l\nWHERE "
              "{\n  {\n    ?c skos:prefLabel \"") +
          name +
          std::string("\"@en.\n  }\n  UNION {\n    ?c skos:altLabel ?l FILTER "
                      "(str(?l) = \"") +
          name + std::string("\").\n  }\n}");

      // Set the Fuseki server URL and the dataset name
      std::string fusekiURL = "http://agrigaia-ur.ni.dfki:3030/plants/query";

      // Set the request parameters
      std::string postData = std::string("query=").append(
          curl_easy_escape(curl, sparqlQuery.c_str(), sparqlQuery.length()));

      // Set the HTTP POST headers
      struct curl_slist* headers = nullptr;
      headers =
          curl_slist_append(headers, "Accept: application/sparql-results+json");

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
        throw std::runtime_error("curl_easy_perform() failed: " +
                                 std::string(curl_easy_strerror(res)));
      }
      else
      {
        Json::Value root;
        Json::Reader reader;
        bool parsingSuccessful = reader.parse(response.c_str(), root);

        if (parsingSuccessful)
        {
          concept = root.get("results", name)
                        .get("bindings", name)[0]
                        .get("c", name)
                        .get("value", name)
                        .asString();
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

std::string getHDF5FilePath(ros::NodeHandle privateNh)
{
  std::string hdf5FolderPath;
  if (!privateNh.getParam("hdf5FolderPath", hdf5FolderPath))
  {
    ROS_WARN_STREAM(
        "Use the \"hdf5FolderPath\" parameter to specify the HDF5 file!");
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
      projectUuid =
          boost::lexical_cast<std::string>(boost::uuids::random_generator()());
      ROS_WARN_STREAM(
          "The provided UUID is invalid! Generating a a new one. (" +
          projectUuid + ".h5)");
    }
  }
  else
  {
    projectUuid =
        boost::lexical_cast<std::string>(boost::uuids::random_generator()());
    ROS_WARN_STREAM("Use the \"projectUuid\" parameter to specify the HDF5 "
                    "file! Generating a a new one. (" +
                    projectUuid + ".h5)");
  }

  return hdf5FolderPath + "/" + projectUuid + ".h5";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seerep_ros_communication_JsonPointDumper");
  ros::NodeHandle privateNh("~");

  std::string jsonFilePath, dataSource, classesMappingPath;

  const std::string hdf5FilePath = getHDF5FilePath(privateNh);

  if (privateNh.getParam("jsonFilePath", jsonFilePath),
      privateNh.getParam("dataSource", dataSource),
      privateNh.getParam("classesMappingPath", classesMappingPath))
  {
    {
      ROS_INFO_STREAM("hdf5FilePath: " << hdf5FilePath);
      ROS_INFO_STREAM("jsonFilePath: " << jsonFilePath);
      ROS_INFO_STREAM("dataSource: " << dataSource);
      ROS_INFO_STREAM("classesMappingPath: " << classesMappingPath);

      seerep_grpc_ros::JsonPointDumper JsonPointDumper(
          jsonFilePath, hdf5FilePath, dataSource, classesMappingPath);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Not all mandatory parameters are set!");
  }

  return EXIT_SUCCESS;
}
