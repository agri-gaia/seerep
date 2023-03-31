#include "seerep_core_fb/core_fb_conversion.h"

namespace seerep_core_fb
{
seerep_core_msgs::Query CoreFbConversion::fromFb(const seerep::fb::QueryInstance* queryInstance)
{
  return seerep_core_fb::CoreFbConversion::fromFb(queryInstance->query(), fromFb(queryInstance->datatype()));
}

seerep_core_msgs::Datatype CoreFbConversion::fromFb(const seerep::fb::Datatype& datatype)
{
  if (datatype == seerep::fb::Datatype_Image)
  {
    return seerep_core_msgs::Datatype::Image;
  }
  else if (datatype == seerep::fb::Datatype_PointCloud)
  {
    return seerep_core_msgs::Datatype::PointCloud;
  }
  else if (datatype == seerep::fb::Datatype_Point)
  {
    return seerep_core_msgs::Datatype::Point;
  }
  else
  {
    return seerep_core_msgs::Datatype::Unknown;
  }
}

seerep_core_msgs::Query CoreFbConversion::fromFb(const seerep::fb::Query* query,
                                                 const seerep_core_msgs::Datatype& datatype)
{
  seerep_core_msgs::Query queryCore;
  queryCore.header.datatype = datatype;

  fromFbQueryBoundingBox(query, queryCore.boundingbox, queryCore.header.frameId);
  fromFbQueryTime(query, queryCore.timeinterval);
  fromFbQueryLabel(query, queryCore.label);
  queryCore.mustHaveAllLabels = fromFbQueryMustHaveAllLabels(query);
  fromFbQueryProject(query, queryCore.projects);
  fromFbQueryInstance(query, queryCore.instances);
  fromFbQueryDataUuids(query, queryCore.dataUuids);
  queryCore.withoutData = fromFbQueryWithoutData(query);
  queryCore.maxNumData = fromFbQueryMaxNumData(query);

  return queryCore;
}

seerep_core_msgs::DatasetIndexable CoreFbConversion::fromFb(const seerep::fb::Image& img)
{
  seerep_core_msgs::DatasetIndexable dataForIndices;

  fromFbDataHeader(img.header(), dataForIndices.header, seerep_core_msgs::Datatype::Image);

  // set bounding box for images to 0. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(0);
  dataForIndices.boundingbox.min_corner().set<1>(0);
  dataForIndices.boundingbox.min_corner().set<2>(0);
  dataForIndices.boundingbox.max_corner().set<0>(0);
  dataForIndices.boundingbox.max_corner().set<1>(0);
  dataForIndices.boundingbox.max_corner().set<2>(0);

  // semantic
  if (flatbuffers::IsFieldPresent(&img, seerep::fb::Image::VT_LABELS_GENERAL))
  {
    fromFbDataLabelsGeneral(img.labels_general(), dataForIndices.labelsWithInstancesWithCategory);
  }
  if (flatbuffers::IsFieldPresent(&img, seerep::fb::Image::VT_LABELS_BB))
  {
    fromFbDataLabelsBb2d(img.labels_bb(), dataForIndices.labelsWithInstancesWithCategory);
  }
  return dataForIndices;
}
seerep_core_msgs::DatasetIndexable CoreFbConversion::fromFb(const seerep::fb::PointStamped* point)
{
  seerep_core_msgs::DatasetIndexable dataForIndices;

  fromFbDataHeader(point->header(), dataForIndices.header, seerep_core_msgs::Datatype::Point);

  // set bounding box for point to point coordinates. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(point->point()->x());
  dataForIndices.boundingbox.min_corner().set<1>(point->point()->y());
  dataForIndices.boundingbox.min_corner().set<2>(point->point()->z());
  dataForIndices.boundingbox.max_corner().set<0>(point->point()->x());
  dataForIndices.boundingbox.max_corner().set<1>(point->point()->y());
  dataForIndices.boundingbox.max_corner().set<2>(point->point()->z());

  // semantic

  if (flatbuffers::IsFieldPresent(point, seerep::fb::PointStamped::VT_LABELS_GENERAL))
  {
    fromFbDataLabelsGeneral(point->labels_general(), dataForIndices.labelsWithInstancesWithCategory);
  }

  return dataForIndices;
}

seerep_core_msgs::DatasetIndexable CoreFbConversion::fromFb(const seerep::fb::PointCloud2& cloud)
{
  seerep_core_msgs::DatasetIndexable dataForIndices;
  fromFbDataHeader(cloud.header(), dataForIndices.header, seerep_core_msgs::Datatype::PointCloud);

  // semantic
  if (flatbuffers::IsFieldPresent(&cloud, seerep::fb::PointCloud2::VT_LABELS_GENERAL))
  {
    fromFbDataLabelsGeneral(cloud.labels_general(), dataForIndices.labelsWithInstancesWithCategory);
  }
  if (flatbuffers::IsFieldPresent(&cloud, seerep::fb::PointCloud2::VT_LABELS_BB))
  {
    fromFbDataLabelsBb(cloud.labels_bb(), dataForIndices.labelsWithInstancesWithCategory);
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

void CoreFbConversion::fromFbQueryProject(const seerep::fb::Query* query,
                                          std::optional<std::vector<boost::uuids::uuid>>& queryCoreProjects)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_PROJECTUUID))
  {
    boost::uuids::string_generator gen;
    queryCoreProjects = std::vector<boost::uuids::uuid>();
    for (auto projectuuid : *query->projectuuid())
    {
      queryCoreProjects.value().push_back(gen(projectuuid->str()));
    }
  }
}

void CoreFbConversion::fromFbQueryInstance(const seerep::fb::Query* query,
                                           std::optional<std::vector<boost::uuids::uuid>>& queryCoreInstances)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_INSTANCEUUID))
  {
    boost::uuids::string_generator gen;
    queryCoreInstances = std::vector<boost::uuids::uuid>();
    for (auto instanceuuid : *query->instanceuuid())
    {
      queryCoreInstances.value().push_back(gen(instanceuuid->str()));
    }
  }
}

void CoreFbConversion::fromFbQueryDataUuids(const seerep::fb::Query* query,
                                            std::optional<std::vector<boost::uuids::uuid>>& queryCoreDataUuids)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_DATAUUID))
  {
    boost::uuids::string_generator gen;
    queryCoreDataUuids = std::vector<boost::uuids::uuid>();
    for (auto datauuid : *query->datauuid())
    {
      queryCoreDataUuids.value().push_back(gen(datauuid->str()));
    }
  }
}

void CoreFbConversion::fromFbQueryLabel(
    const seerep::fb::Query* query,
    std::optional<std::unordered_map<std::string, std::vector<std::string>>>& queryCoreLabel)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_LABEL))
  {
    queryCoreLabel = std::unordered_map<std::string, std::vector<std::string>>();
    for (auto labelWithCategory : *query->label())
    {
      std::vector<std::string> labels;
      for (auto label : *labelWithCategory->labels())
      {
        labels.push_back(label->label()->str());
      }
      queryCoreLabel.value().emplace(labelWithCategory->category()->c_str(), labels);
    }
  }
}

void CoreFbConversion::fromFbQueryTime(const seerep::fb::Query* query,
                                       std::optional<seerep_core_msgs::Timeinterval>& queryCoreTime)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_TIMEINTERVAL))
  {
    queryCoreTime = seerep_core_msgs::Timeinterval();
    queryCoreTime.value().timeMin.seconds = query->timeinterval()->time_min()->seconds();
    queryCoreTime.value().timeMax.seconds = query->timeinterval()->time_max()->seconds();
    queryCoreTime.value().timeMin.nanos = query->timeinterval()->time_min()->nanos();
    queryCoreTime.value().timeMax.nanos = query->timeinterval()->time_max()->nanos();
  }
}

void CoreFbConversion::fromFbQueryBoundingBox(const seerep::fb::Query* query,
                                              std::optional<seerep_core_msgs::AABB>& queryCoreBoundingBox,
                                              std::string& queryCoreHeaderFrameId)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_BOUNDINGBOXSTAMPED))
  {
    queryCoreHeaderFrameId = query->boundingboxStamped()->header()->frame_id()->str();
    queryCoreBoundingBox = seerep_core_msgs::AABB();
    queryCoreBoundingBox.value().min_corner().set<0>(query->boundingboxStamped()->boundingbox()->center_point()->x() -
                                                     query->boundingboxStamped()->boundingbox()->spatial_extent()->x() /
                                                         2.0);
    queryCoreBoundingBox.value().min_corner().set<1>(query->boundingboxStamped()->boundingbox()->center_point()->y() -
                                                     query->boundingboxStamped()->boundingbox()->spatial_extent()->y() /
                                                         2.0);
    queryCoreBoundingBox.value().min_corner().set<2>(query->boundingboxStamped()->boundingbox()->center_point()->z() -
                                                     query->boundingboxStamped()->boundingbox()->spatial_extent()->z() /
                                                         2.0);
    queryCoreBoundingBox.value().max_corner().set<0>(query->boundingboxStamped()->boundingbox()->center_point()->x() +
                                                     query->boundingboxStamped()->boundingbox()->spatial_extent()->x() /
                                                         2.0);
    queryCoreBoundingBox.value().max_corner().set<1>(query->boundingboxStamped()->boundingbox()->center_point()->y() +
                                                     query->boundingboxStamped()->boundingbox()->spatial_extent()->y() /
                                                         2.0);
    queryCoreBoundingBox.value().max_corner().set<2>(query->boundingboxStamped()->boundingbox()->center_point()->z() +
                                                     query->boundingboxStamped()->boundingbox()->spatial_extent()->z() /
                                                         2.0);
  }
}

bool CoreFbConversion::fromFbQueryWithoutData(const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_WITHOUTDATA))
  {
    return query->withoutdata();
  }

  return false;
}

bool CoreFbConversion::fromFbQueryMustHaveAllLabels(const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_MUSTHAVEALLLABELS))
  {
    return query->mustHaveAllLabels();
  }

  return false;
}

uint CoreFbConversion::fromFbQueryMaxNumData(const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_MAXNUMDATA))
  {
    return query->maxNumData();
  }

  return 0;
}

void CoreFbConversion::fromFbDataHeader(const seerep::fb::Header* header, seerep_core_msgs::Header& coreHeader,
                                        seerep_core_msgs::Datatype&& datatype)
{
  if (flatbuffers::IsFieldPresent(header, seerep::fb::Header::VT_UUID_MSGS))
  {
    coreHeader.uuidData = fromFbDataHeaderUuid(header->uuid_msgs()->str());
  }
  else
  {
    coreHeader.uuidData = boost::uuids::random_generator()();
  }

  coreHeader.datatype = datatype;
  coreHeader.frameId = header->frame_id()->str();
  coreHeader.timestamp.seconds = header->stamp()->seconds();
  coreHeader.timestamp.nanos = header->stamp()->nanos();

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
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>>* labelsGeneral,
    std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory)
{
  if (labelsGeneral)
  {
    for (auto labelsCategories : *labelsGeneral)
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      if (labelsCategories->labelsWithInstance())
      {
        labelWithInstanceVector.reserve(labelsCategories->labelsWithInstance()->size());
        for (auto label : *labelsCategories->labelsWithInstance())
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

          labelWithInstanceVector.push_back(
              seerep_core_msgs::LabelWithInstance{ .label = label->label()->label()->str(),
                                                   .labelConfidence = label->label()->confidence(),
                                                   .uuidInstance = uuidInstance });
        }
        labelsWithInstancesWithCategory.emplace(labelsCategories->category()->c_str(), labelWithInstanceVector);
      }
    }
  }
}

void CoreFbConversion::fromFbDataLabelsBb2d(
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>>* labelsBB2d,
    std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory)
{
  if (labelsBB2d)
  {
    for (auto labelsCategories : *labelsBB2d)
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      for (auto label : *labelsCategories->boundingBox2dLabeled())
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

        labelWithInstanceVector.push_back(
            seerep_core_msgs::LabelWithInstance{ .label = label->labelWithInstance()->label()->label()->str(),
                                                 .labelConfidence = label->labelWithInstance()->label()->confidence(),
                                                 .uuidInstance = uuidInstance });
      }
      labelsWithInstancesWithCategory.emplace(labelsCategories->category()->c_str(), labelWithInstanceVector);
    }
  }
}

void CoreFbConversion::fromFbDataLabelsBb(
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeledWithCategory>>* labelsBB,
    std::unordered_map<std::string, std::vector<seerep_core_msgs::LabelWithInstance>>& labelsWithInstancesWithCategory)
{
  if (labelsBB)
  {
    for (auto labelsCategories : *labelsBB)
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      for (auto label : *labelsCategories->boundingBoxLabeled())
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

        labelWithInstanceVector.push_back(
            seerep_core_msgs::LabelWithInstance{ .label = label->labelWithInstance()->label()->label()->str(),
                                                 .labelConfidence = label->labelWithInstance()->label()->confidence(),
                                                 .uuidInstance = uuidInstance });
      }
      labelsWithInstancesWithCategory.emplace(labelsCategories->category()->c_str(), labelWithInstanceVector);
    }
  }
}
}  // namespace seerep_core_fb