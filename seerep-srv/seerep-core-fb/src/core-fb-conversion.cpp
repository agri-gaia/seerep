#include "seerep-core-fb/core-fb-conversion.h"

namespace seerep_core_fb
{
seerep_core_msgs::Query CoreFbConversion::fromFb(const seerep::fb::Query& query)
{
  seerep_core_msgs::Query queryCore;
  queryCore.header.datatype = seerep_core_msgs::Datatype::Images;

  fromFbProject(query, queryCore);
  fromFbLabel(query, queryCore);
  fromFbTime(query, queryCore);
  fromFbBoundingBox(query, queryCore);

  return queryCore;
}

seerep_core_msgs::DatasetIndexable CoreFbConversion::fromFb(const seerep::fb::Image& img)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid;
  if (img.header()->uuid_msgs()->str().empty())
  {
    uuid = boost::uuids::random_generator()();
  }
  else
  {
    uuid = gen(img.header()->uuid_msgs()->str());
  }

  seerep_core_msgs::DatasetIndexable dataForIndices;
  dataForIndices.header.datatype = seerep_core_msgs::Datatype::Images;
  dataForIndices.header.frameId = img.header()->frame_id()->str();
  dataForIndices.header.timestamp.seconds = img.header()->stamp()->seconds();
  dataForIndices.header.timestamp.nanos = img.header()->stamp()->nanos();
  dataForIndices.header.uuidData = uuid;
  dataForIndices.header.uuidProject = gen(img.header()->uuid_project()->str());
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
  if (img.labels_general())
  {
    for (auto label : *img.labels_general())
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

      dataForIndices.labelsWithInstances.push_back(
          seerep_core_msgs::LabelWithInstance{ .label = label->label()->str(), .uuidInstance = uuidInstance });
    }
  }

  if (img.labels_bb())
  {
    for (auto label : *img.labels_bb())
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

      dataForIndices.labelsWithInstances.push_back(seerep_core_msgs::LabelWithInstance{
          .label = label->labelWithInstance()->label()->str(), .uuidInstance = uuidInstance });
    }
  }

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

void CoreFbConversion::fromFbProject(const seerep::fb::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (query.projectuuid() != NULL)
  {
    queryCore.projects = std::vector<boost::uuids::uuid>();
    for (auto projectuuid : *query.projectuuid())
    {
      queryCore.projects.value().push_back(gen(projectuuid->str()));
    }
  }
}

void CoreFbConversion::fromFbLabel(const seerep::fb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.label() != NULL)
  {
    queryCore.label = std::vector<std::string>();
    for (auto label : *query.label())
    {
      queryCore.label.value().push_back(label->str());
    }
  }
}

void CoreFbConversion::fromFbTime(const seerep::fb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.timeinterval() != NULL)
  {
    queryCore.timeinterval = seerep_core_msgs::Timeinterval();
    queryCore.timeinterval.value().timeMin.seconds = query.timeinterval()->time_min()->seconds();
    queryCore.timeinterval.value().timeMax.seconds = query.timeinterval()->time_max()->seconds();
    queryCore.timeinterval.value().timeMin.nanos = query.timeinterval()->time_min()->nanos();
    queryCore.timeinterval.value().timeMax.nanos = query.timeinterval()->time_max()->nanos();
  }
}

void CoreFbConversion::fromFbBoundingBox(const seerep::fb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.boundingbox() != NULL)
  {
    queryCore.header.frameId = query.boundingbox()->header()->frame_id()->str();
    queryCore.boundingbox = seerep_core_msgs::AABB();
    queryCore.boundingbox.value().min_corner().set<0>(query.boundingbox()->point_min()->x());
    queryCore.boundingbox.value().min_corner().set<1>(query.boundingbox()->point_min()->y());
    queryCore.boundingbox.value().min_corner().set<2>(query.boundingbox()->point_min()->z());
    queryCore.boundingbox.value().max_corner().set<0>(query.boundingbox()->point_max()->x());
    queryCore.boundingbox.value().max_corner().set<1>(query.boundingbox()->point_max()->y());
    queryCore.boundingbox.value().max_corner().set<2>(query.boundingbox()->point_max()->z());
  }
}
}  // namespace seerep_core_fb
