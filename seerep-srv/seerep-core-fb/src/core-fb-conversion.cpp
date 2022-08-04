#include "seerep-core-fb/core-fb-conversion.h"

namespace seerep_core_fb
{
seerep_core_msgs::Query CoreFbConversion::fromFb(const seerep::fb::Query& query)
{
  seerep_core_msgs::Query queryCore;
  queryCore.header.datatype = seerep_core_msgs::Datatype::Images;

  fromFbQueryProject(query.projectuuid(), queryCore.projects);
  fromFbQueryLabel(query.label(), queryCore.label);
  fromFbQueryTime(query.timeinterval(), queryCore.timeinterval);
  fromFbQueryBoundingBox(query.boundingbox(), queryCore.boundingbox, queryCore.header.frameId);

  return queryCore;
}

seerep_core_msgs::DatasetIndexable CoreFbConversion::fromFb(const seerep::fb::Image& img)
{
  seerep_core_msgs::DatasetIndexable dataForIndices;

  fromFbDataHeader(img.header(), dataForIndices.header);

  // set bounding box for images to 0. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(0);
  dataForIndices.boundingbox.min_corner().set<1>(0);
  dataForIndices.boundingbox.min_corner().set<2>(0);
  dataForIndices.boundingbox.max_corner().set<0>(0);
  dataForIndices.boundingbox.max_corner().set<1>(0);
  dataForIndices.boundingbox.max_corner().set<2>(0);

  // semantic
  int labelSizeAll = 0;
  if (img.labels_general())
  {
    labelSizeAll += img.labels_general()->size();
  }
  if (img.labels_bb())
  {
    labelSizeAll += img.labels_bb()->size();
  }
  dataForIndices.labelsWithInstances.reserve(labelSizeAll);

  fromFbDataLabelsGeneral(img.labels_general(), dataForIndices.labelsWithInstances);
  fromFbDataLabelsGeneral(img.labels_bb(), dataForIndices.labelsWithInstances);

  return dataForIndices;
}
seerep_core_msgs::DatasetIndexable CoreFbConversion::fromFb(const seerep::fb::PointStamped& point)
{
  seerep_core_msgs::DatasetIndexable dataForIndices;

  fromFbDataHeader(point.header(), dataForIndices.header);

  // set bounding box for point to point coordinates. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(point.point()->x());
  dataForIndices.boundingbox.min_corner().set<1>(point.point()->y());
  dataForIndices.boundingbox.min_corner().set<2>(point.point()->z());
  dataForIndices.boundingbox.max_corner().set<0>(point.point()->x());
  dataForIndices.boundingbox.max_corner().set<1>(point.point()->y());
  dataForIndices.boundingbox.max_corner().set<2>(point.point()->z());

  // semantic
  int labelSizeAll = 0;
  if (point.labels_general())
  {
    labelSizeAll += point.labels_general()->size();
  }

  dataForIndices.labelsWithInstances.reserve(labelSizeAll);

  fromFbDataLabelsGeneral(point.labels_general(), dataForIndices.labelsWithInstances);

  return dataForIndices;
}

seerep_core_msgs::QueryTf CoreFbConversion::fromFb(const seerep::fb::TransformStampedQuery& query)
{
  boost::uuids::string_generator gen;
  seerep_core_msgs::QueryTf queryTf;
  queryTf.childFrameId = query.child_frame_id()->str();
  queryTf.parentFrameId = query.header()->frame_id()->str();
  queryTf.project = gen(query.header()->uuid_project()->str());
  queryTf.timestamp.seconds = query.header()->stamp()->seconds();
  queryTf.timestamp.nanos = query.header()->stamp()->nanos();

  return queryTf;
}

flatbuffers::grpc::Message<seerep::fb::UuidsPerProject> CoreFbConversion::toFb(seerep_core_msgs::QueryResult& result)
{
  flatbuffers::grpc::MessageBuilder builder;

  std::vector<flatbuffers::Offset<seerep::fb::UuidsProjectPair>> uuidsPerProject;
  uuidsPerProject.reserve(result.queryResultProjects.size());
  for (auto projectResult : result.queryResultProjects)
  {
    std::vector<flatbuffers::Offset<flatbuffers::String>> uuids;
    uuids.reserve(projectResult.dataOrInstanceUuids.size());
    for (auto uuid : projectResult.dataOrInstanceUuids)
    {
      uuids.push_back(builder.CreateString(boost::lexical_cast<std::string>(uuid)));
    }

    auto uuidsVector = builder.CreateVector(uuids);
    auto projectUuid = builder.CreateString(boost::lexical_cast<std::string>(projectResult.projectUuid));

    seerep::fb::UuidsProjectPairBuilder uuidsProjectPairBuilder(builder);
    uuidsProjectPairBuilder.add_projectUuid(projectUuid);
    uuidsProjectPairBuilder.add_uuids(uuidsVector);
    uuidsPerProject.push_back(uuidsProjectPairBuilder.Finish());
  }
  auto uuidsPerProjectVector = builder.CreateVector(uuidsPerProject);
  seerep::fb::UuidsPerProjectBuilder uuidsPerProjectBuilder(builder);
  uuidsPerProjectBuilder.add_uuidsPerProject(uuidsPerProjectVector);

  builder.Finish(uuidsPerProjectBuilder.Finish());
  return builder.ReleaseMessage<seerep::fb::UuidsPerProject>();
}

void CoreFbConversion::fromFbQueryProject(const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>* projects,
                                          std::optional<std::vector<boost::uuids::uuid>>& queryCoreProjects)
{
  boost::uuids::string_generator gen;
  if (projects != NULL)
  {
    queryCoreProjects = std::vector<boost::uuids::uuid>();
    for (auto projectuuid : *projects)
    {
      queryCoreProjects.value().push_back(gen(projectuuid->str()));
    }
  }
}

void CoreFbConversion::fromFbQueryLabel(const flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>* label,
                                        std::optional<std::vector<std::string>>& queryCoreLabel)
{
  if (label != NULL)
  {
    queryCoreLabel = std::vector<std::string>();
    for (auto label : *label)
    {
      queryCoreLabel.value().push_back(label->str());
    }
  }
}

void CoreFbConversion::fromFbQueryTime(const seerep::fb::TimeInterval* time,
                                       std::optional<seerep_core_msgs::Timeinterval>& queryCoreTime)
{
  if (time != NULL)
  {
    queryCoreTime = seerep_core_msgs::Timeinterval();
    queryCoreTime.value().timeMin.seconds = time->time_min()->seconds();
    queryCoreTime.value().timeMax.seconds = time->time_max()->seconds();
    queryCoreTime.value().timeMin.nanos = time->time_min()->nanos();
    queryCoreTime.value().timeMax.nanos = time->time_max()->nanos();
  }
}

void CoreFbConversion::fromFbQueryBoundingBox(const seerep::fb::Boundingbox* boundingBox,
                                              std::optional<seerep_core_msgs::AABB>& queryCoreBoundingBox,
                                              std::string& queryCoreHeaderFrameId)
{
  if (boundingBox != NULL)
  {
    queryCoreHeaderFrameId = boundingBox->header()->frame_id()->str();
    queryCoreBoundingBox = seerep_core_msgs::AABB();
    queryCoreBoundingBox.value().min_corner().set<0>(boundingBox->point_min()->x());
    queryCoreBoundingBox.value().min_corner().set<1>(boundingBox->point_min()->y());
    queryCoreBoundingBox.value().min_corner().set<2>(boundingBox->point_min()->z());
    queryCoreBoundingBox.value().max_corner().set<0>(boundingBox->point_max()->x());
    queryCoreBoundingBox.value().max_corner().set<1>(boundingBox->point_max()->y());
    queryCoreBoundingBox.value().max_corner().set<2>(boundingBox->point_max()->z());
  }
}

void CoreFbConversion::fromFbDataHeader(const seerep::fb::Header* header, seerep_core_msgs::Header& coreHeader)
{
  boost::uuids::uuid uuid = fromFbDataHeaderUuid(header->uuid_msgs()->str());

  coreHeader.datatype = seerep_core_msgs::Datatype::Images;
  coreHeader.frameId = header->frame_id()->str();
  coreHeader.timestamp.seconds = header->stamp()->seconds();
  coreHeader.timestamp.nanos = header->stamp()->nanos();
  coreHeader.uuidData = uuid;

  boost::uuids::string_generator gen;
  coreHeader.uuidProject = gen(header->uuid_project()->str());
}

boost::uuids::uuid CoreFbConversion::fromFbDataHeaderUuid(const std::string& uuidMsg)
{
  boost::uuids::string_generator gen;

  if (uuidMsg.empty())
  {
    return boost::uuids::random_generator()();
  }
  else
  {
    return gen(uuidMsg);
  }
}

void CoreFbConversion::fromFbDataLabelsGeneral(
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>* labelsGeneral,
    std::vector<seerep_core_msgs::LabelWithInstance>& labelWithInstance)
{
  if (labelsGeneral)
  {
    for (auto label : *labelsGeneral)
    {
      boost::uuids::string_generator gen;
      boost::uuids::uuid uuidInstance;
      try
      {
        uuidInstance = gen(label->instanceUuid()->str());
      }
      catch (std::runtime_error const& e)
      {
        uuidInstance = boost::uuids::nil_uuid();
      }

      labelWithInstance.push_back(
          seerep_core_msgs::LabelWithInstance{ .label = label->label()->str(), .uuidInstance = uuidInstance });
    }
  }
}

void CoreFbConversion::fromFbDataLabelsGeneral(
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>>* labelsBB2d,
    std::vector<seerep_core_msgs::LabelWithInstance>& labelWithInstance)
{
  if (labelsBB2d)
  {
    for (auto label : *labelsBB2d)
    {
      boost::uuids::string_generator gen;
      boost::uuids::uuid uuidInstance;
      try
      {
        uuidInstance = gen(label->labelWithInstance()->instanceUuid()->str());
      }
      catch (std::runtime_error const& e)
      {
        uuidInstance = boost::uuids::nil_uuid();
      }

      labelWithInstance.push_back(seerep_core_msgs::LabelWithInstance{
          .label = label->labelWithInstance()->label()->str(), .uuidInstance = uuidInstance });
    }
  }
}
}  // namespace seerep_core_fb
