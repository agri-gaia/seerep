#include "seerep_core_fb/core_fb_conversion.h"

namespace seerep_core_fb
{
seerep_core_msgs::Query
CoreFbConversion::fromFb(const seerep::fb::QueryInstance* queryInstance)
{
  return seerep_core_fb::CoreFbConversion::fromFb(
      queryInstance->query(), fromFb(queryInstance->datatype()));
}

seerep_core_msgs::Datatype
CoreFbConversion::fromFb(const seerep::fb::Datatype& datatype)
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

seerep_core_msgs::Query
CoreFbConversion::fromFb(const seerep::fb::Query* query,
                         const seerep_core_msgs::Datatype& datatype)
{
  seerep_core_msgs::Query queryCore;
  queryCore.header.datatype = datatype;

  fromFbQueryTime(query, queryCore.timeintervals);
  fromFbQueryLabel(query, queryCore.label);
  queryCore.sparqlQuery = fromFbSparqlQuery(query);
  queryCore.ontologyURI = fromFbOntologyURI(query);
  queryCore.mustHaveAllLabels = fromFbQueryMustHaveAllLabels(query);
  fromFbQueryProject(query, queryCore.projects);
  fromFbQueryInstance(query, queryCore.instances);
  fromFbQueryDataUuids(query, queryCore.dataUuids);
  queryCore.withoutData = fromFbQueryWithoutData(query);
  queryCore.maxNumData = fromFbQueryMaxNumData(query);
  queryCore.polygon = fromFbQueryPolygon(query);
  queryCore.polygonSensorPos = fromFbQueryPolygonSensorPosition(query);
  queryCore.fullyEncapsulated = fromFbQueryFullyEncapsulated(query);
  queryCore.inMapFrame = fromFbQueryInMapFrame(query);
  queryCore.sortByTime = query->sortByTime();

  return queryCore;
}

seerep_core_msgs::DatasetIndexable
CoreFbConversion::fromFb(const seerep::fb::Image& img)
{
  seerep_core_msgs::DatasetIndexable dataForIndices;

  fromFbDataHeader(img.header(), dataForIndices.header,
                   seerep_core_msgs::Datatype::Image);

  // set bounding box for images to 0. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(0);
  dataForIndices.boundingbox.min_corner().set<1>(0);
  dataForIndices.boundingbox.min_corner().set<2>(0);
  dataForIndices.boundingbox.max_corner().set<0>(0);
  dataForIndices.boundingbox.max_corner().set<1>(0);
  dataForIndices.boundingbox.max_corner().set<2>(0);

  // semantic
  if (flatbuffers::IsFieldPresent(&img, seerep::fb::Image::VT_LABELS))
  {
    fromFbDataLabels(img.labels(), dataForIndices.labelsCategory);
  }

  return dataForIndices;
}
seerep_core_msgs::DatasetIndexable
CoreFbConversion::fromFb(const seerep::fb::PointStamped* point)
{
  seerep_core_msgs::DatasetIndexable dataForIndices;

  fromFbDataHeader(point->header(), dataForIndices.header,
                   seerep_core_msgs::Datatype::Point);

  // set bounding box for point to point coordinates. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(point->point()->x());
  dataForIndices.boundingbox.min_corner().set<1>(point->point()->y());
  dataForIndices.boundingbox.min_corner().set<2>(point->point()->z());
  dataForIndices.boundingbox.max_corner().set<0>(point->point()->x());
  dataForIndices.boundingbox.max_corner().set<1>(point->point()->y());
  dataForIndices.boundingbox.max_corner().set<2>(point->point()->z());

  // semantic

  if (flatbuffers::IsFieldPresent(point, seerep::fb::PointStamped::VT_LABELS))
  {
    fromFbDataLabels(point->labels(), dataForIndices.labelsCategory);
  }

  return dataForIndices;
}

seerep_core_msgs::DatasetIndexable
CoreFbConversion::fromFb(const seerep::fb::PointCloud2& cloud)
{
  seerep_core_msgs::DatasetIndexable dataForIndices;
  fromFbDataHeader(cloud.header(), dataForIndices.header,
                   seerep_core_msgs::Datatype::PointCloud);

  // semantic
  if (flatbuffers::IsFieldPresent(&cloud, seerep::fb::PointCloud2::VT_LABELS))
  {
    fromFbDataLabels(cloud.labels(), dataForIndices.labelsCategory);
  }
  return dataForIndices;
}

seerep_core_msgs::QueryTf
CoreFbConversion::fromFb(const seerep::fb::TransformStampedQuery& query)
{
  boost::uuids::string_generator gen;
  seerep_core_msgs::QueryTf queryTf;
  queryTf.childFrameId = query.child_frame_id()->str();
  queryTf.parentFrameId = query.header()->frame_id()->str();
  if (flatbuffers::IsFieldPresent(query.header(),
                                  seerep::fb::Header::VT_UUID_PROJECT))
  {
    queryTf.project = gen(query.header()->uuid_project()->str());
  }
  else
  {
    queryTf.project = boost::uuids::nil_uuid();
  }
  queryTf.timestamp.seconds = query.header()->stamp()->seconds();
  queryTf.timestamp.nanos = query.header()->stamp()->nanos();

  return queryTf;
}

seerep_core_msgs::camera_intrinsics
CoreFbConversion::fromFb(const seerep::fb::CameraIntrinsics& ci)
{
  seerep_core_msgs::camera_intrinsics ciCore;

  CoreFbConversion::fromFbDataHeader(ci.header(), ciCore.header,
                                     seerep_core_msgs::Datatype::Unknown);

  ciCore.height = ci.height();
  ciCore.width = ci.width();
  ciCore.distortion_model = ci.distortion_model()->str();

  // traverse distortion list and convert
  if (flatbuffers::IsFieldPresent(&ci,
                                  seerep::fb::CameraIntrinsics::VT_DISTORTION))
  {
    for (auto distortion_val : *ci.distortion())
    {
      ciCore.distortion.push_back(distortion_val);
    }
  }

  // traverse intrinsics matrix and convert
  if (flatbuffers::IsFieldPresent(
          &ci, seerep::fb::CameraIntrinsics::VT_INTRINSIC_MATRIX))
  {
    for (auto intrinsic_matrix_elem : *ci.intrinsic_matrix())
    {
      ciCore.intrinsic_matrix.push_back(intrinsic_matrix_elem);
    }
  }

  // traverse rectification matrix and convert
  if (flatbuffers::IsFieldPresent(
          &ci, seerep::fb::CameraIntrinsics::VT_RECTIFICATION_MATRIX))
  {
    for (auto rectification_matrix_elem : *ci.rectification_matrix())
    {
      ciCore.rectification_matrix.push_back(rectification_matrix_elem);
    }
  }

  // traverse projection matrix and convert
  if (flatbuffers::IsFieldPresent(
          &ci, seerep::fb::CameraIntrinsics::VT_PROJECTION_MATRIX))
  {
    for (auto projection_matrix_elem : *ci.projection_matrix())
    {
      ciCore.projection_matrix.push_back(projection_matrix_elem);
    }
  }

  ciCore.binning_x = ci.binning_x();
  ciCore.binning_y = ci.binning_y();

  ciCore.region_of_interest =
      CoreFbConversion::fromFb(*ci.region_of_interest());

  ciCore.maximum_viewing_distance = ci.maximum_viewing_distance();

  return ciCore;
}

seerep_core_msgs::region_of_interest
CoreFbConversion::fromFb(const seerep::fb::RegionOfInterest& roi)
{
  seerep_core_msgs::region_of_interest roiCore;

  roiCore.x_offset = roi.x_offset();
  roiCore.y_offset = roi.y_offset();
  roiCore.height = roi.height();
  roiCore.width = roi.width();
  roiCore.do_rectify = roi.do_rectify();

  return roiCore;
}

flatbuffers::Offset<seerep::fb::RegionOfInterest>
CoreFbConversion::toFb(flatbuffers::grpc::MessageBuilder& mb,
                       const seerep_core_msgs::region_of_interest& roi)
{
  seerep::fb::RegionOfInterestBuilder roi_builder(mb);

  roi_builder.add_x_offset(roi.x_offset);
  roi_builder.add_y_offset(roi.y_offset);
  roi_builder.add_height(roi.height);
  roi_builder.add_width(roi.width);
  roi_builder.add_do_rectify(roi.do_rectify);

  return roi_builder.Finish();
}

flatbuffers::Offset<seerep::fb::Timestamp>
CoreFbConversion::toFb(flatbuffers::grpc::MessageBuilder& mb,
                       const seerep_core_msgs::Timestamp ts)
{
  seerep::fb::TimestampBuilder tsb(mb);

  tsb.add_nanos(ts.nanos);
  tsb.add_seconds(ts.seconds);

  return tsb.Finish();
}

flatbuffers::Offset<seerep::fb::Header>
CoreFbConversion::toFb(flatbuffers::grpc::MessageBuilder& mb,
                       const seerep_core_msgs::Header header)
{
  auto timeStampMsg = CoreFbConversion::toFb(mb, header.timestamp);
  auto uuidProject =
      mb.CreateString(boost::lexical_cast<std::string>(header.uuidProject));
  auto uuidMsg =
      mb.CreateString(boost::lexical_cast<std::string>(header.uuidData));
  auto frameIdMsg =
      mb.CreateString(boost::lexical_cast<std::string>(header.frameId));

  seerep::fb::HeaderBuilder headerBuilder(mb);
  headerBuilder.add_uuid_msgs(uuidMsg);
  headerBuilder.add_frame_id(frameIdMsg);
  headerBuilder.add_stamp(timeStampMsg);
  headerBuilder.add_uuid_project(uuidProject);
  headerBuilder.add_seq(header.sequence);

  return headerBuilder.Finish();
}

flatbuffers::Offset<seerep::fb::CameraIntrinsics>
CoreFbConversion::toFb(flatbuffers::grpc::MessageBuilder& mb,
                       const seerep_core_msgs::camera_intrinsics ci)
{
  auto dist_model = mb.CreateString(ci.distortion_model);
  auto dist = mb.CreateVector<double>(ci.distortion);
  auto im = mb.CreateVector<double>(ci.intrinsic_matrix);
  auto rm = mb.CreateVector<double>(ci.rectification_matrix);
  auto pm = mb.CreateVector<double>(ci.projection_matrix);

  auto roi = CoreFbConversion::toFb(mb, ci.region_of_interest);

  auto header = CoreFbConversion::toFb(mb, ci.header);

  seerep::fb::CameraIntrinsicsBuilder cib(mb);

  cib.add_header(header);

  cib.add_height(ci.height);
  cib.add_width(ci.width);

  cib.add_distortion_model(dist_model);

  cib.add_distortion(dist);

  cib.add_intrinsic_matrix(im);
  cib.add_rectification_matrix(rm);
  cib.add_projection_matrix(pm);

  cib.add_binning_x(ci.binning_x);
  cib.add_binning_y(ci.binning_y);

  cib.add_region_of_interest(roi);

  cib.add_maximum_viewing_distance(ci.maximum_viewing_distance);

  return cib.Finish();
}

seerep_core_msgs::camera_intrinsics_query CoreFbConversion::fromFb(
    const seerep::fb::CameraIntrinsicsQuery& camIntrinsicsQuery)
{
  seerep_core_msgs::camera_intrinsics_query ciq;

  boost::uuids::string_generator gen;
  ciq.uuidCameraIntrinsics =
      gen(camIntrinsicsQuery.uuid_camera_intrinsics()->str());
  ciq.uuidProject = gen(camIntrinsicsQuery.uuid_project()->str());

  return ciq;
}

flatbuffers::grpc::Message<seerep::fb::UuidsPerProject>
CoreFbConversion::toFb(seerep_core_msgs::QueryResult& result)
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
      uuids.push_back(
          builder.CreateString(boost::lexical_cast<std::string>(uuid)));
    }

    auto uuidsVector = builder.CreateVector(uuids);
    auto projectUuid = builder.CreateString(
        boost::lexical_cast<std::string>(projectResult.projectUuid));

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

flatbuffers::Offset<seerep::fb::Boundingbox>
CoreFbConversion::toFb(flatbuffers::grpc::MessageBuilder& mb,
                       seerep_core_msgs::AABB& aabb)
{
  // center
  float center_x =
      (aabb.min_corner().get<0>() + aabb.max_corner().get<0>()) / 2;
  float center_y =
      (aabb.min_corner().get<1>() + aabb.max_corner().get<1>()) / 2;
  float center_z =
      (aabb.min_corner().get<2>() + aabb.max_corner().get<2>()) / 2;

  // spatial extent
  float se_x = (aabb.max_corner().get<0>() - aabb.min_corner().get<0>());
  float se_y = (aabb.max_corner().get<1>() - aabb.min_corner().get<1>());
  float se_z = (aabb.max_corner().get<2>() - aabb.min_corner().get<2>());

  seerep::fb::PointBuilder centerPointBuilder(mb);
  centerPointBuilder.add_x(center_x);
  centerPointBuilder.add_y(center_y);
  centerPointBuilder.add_z(center_z);
  flatbuffers::Offset<seerep::fb::Point> centerPoint =
      centerPointBuilder.Finish();

  seerep::fb::PointBuilder spatialExtentBuilder(mb);
  spatialExtentBuilder.add_x(se_x);
  spatialExtentBuilder.add_y(se_y);
  spatialExtentBuilder.add_z(se_z);
  flatbuffers::Offset<seerep::fb::Point> spatialExtent =
      spatialExtentBuilder.Finish();

  seerep::fb::BoundingboxBuilder boundingBoxBuilder(mb);
  boundingBoxBuilder.add_center_point(centerPoint);
  boundingBoxBuilder.add_spatial_extent(spatialExtent);

  return boundingBoxBuilder.Finish();
}

flatbuffers::Offset<seerep::fb::TimeInterval>
CoreFbConversion::toFb(flatbuffers::grpc::MessageBuilder& mb,
                       seerep_core_msgs::AabbTime& timeinterval)
{
  // isolate second and nano second bits from min time
  int64_t mintime = timeinterval.min_corner().get<0>();
  int32_t min_nanos = (int32_t)mintime;
  uint32_t min_seconds = (uint32_t)(mintime >> 32);

  // isolate second and nano second bits from max time
  int64_t maxtime = timeinterval.max_corner().get<0>();
  int32_t max_nanos = (int32_t)maxtime;
  uint32_t max_seconds = (uint32_t)(maxtime >> 32);

  seerep::fb::TimestampBuilder minTimeStampBuilder(mb);
  minTimeStampBuilder.add_seconds(min_seconds);
  minTimeStampBuilder.add_nanos(min_nanos);
  flatbuffers::Offset<seerep::fb::Timestamp> min = minTimeStampBuilder.Finish();

  seerep::fb::TimestampBuilder maxTimeStampBuilder(mb);
  maxTimeStampBuilder.add_seconds(max_seconds);
  maxTimeStampBuilder.add_nanos(max_nanos);
  flatbuffers::Offset<seerep::fb::Timestamp> max = maxTimeStampBuilder.Finish();

  seerep::fb::TimeIntervalBuilder timeIntervalBuilder(mb);
  timeIntervalBuilder.add_time_min(min);
  timeIntervalBuilder.add_time_max(max);

  return timeIntervalBuilder.Finish();
}

flatbuffers::Offset<seerep::fb::ProjectInfo>
CoreFbConversion::toFb(flatbuffers::grpc::MessageBuilder& fbb,
                       const seerep_core_msgs::ProjectInfo& prjInfo)
{
  auto geoCordsOffset = seerep::fb::CreateGeodeticCoordinatesDirect(
      fbb, prjInfo.geodetCoords.coordinateSystem.c_str(),
      prjInfo.geodetCoords.longitude, prjInfo.geodetCoords.latitude,
      prjInfo.geodetCoords.altitude);

  return seerep::fb::CreateProjectInfoDirect(
      fbb, prjInfo.name.c_str(),
      boost::lexical_cast<std::string>(prjInfo.uuid).c_str(),
      prjInfo.frameId.c_str(), geoCordsOffset, prjInfo.version.c_str());
}

flatbuffers::Offset<seerep::fb::ProjectInfos> CoreFbConversion::toFb(
    flatbuffers::grpc::MessageBuilder& fbb,
    const std::vector<seerep_core_msgs::ProjectInfo>& prjInfos)
{
  std::vector<flatbuffers::Offset<seerep::fb::ProjectInfo>> prjInfosConv;
  for (auto prjInfo : prjInfos)
  {
    prjInfosConv.push_back(toFb(fbb, prjInfo));
  }
  return seerep::fb::CreateProjectInfosDirect(fbb, &prjInfosConv);
}

void CoreFbConversion::fromFbQueryProject(
    const seerep::fb::Query* query,
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

void CoreFbConversion::fromFbQueryInstance(
    const seerep::fb::Query* query,
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

void CoreFbConversion::fromFbQueryDataUuids(
    const seerep::fb::Query* query,
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
    std::optional<std::unordered_map<std::string, std::vector<std::string>>>&
        queryCoreLabel)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_LABEL))
  {
    queryCoreLabel =
        std::unordered_map<std::string, std::vector<std::string>>();
    for (auto labelWithCategory : *query->label())
    {
      std::vector<std::string> labels;
      for (auto label : *labelWithCategory->labels())
      {
        labels.push_back(label->label()->str());
      }
      queryCoreLabel.value().emplace(labelWithCategory->category()->c_str(),
                                     labels);
    }
  }
}

void CoreFbConversion::fromFbQueryTime(
    const seerep::fb::Query* query,
    std::optional<std::vector<seerep_core_msgs::Timeinterval>>& queryCoreTimes)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_TIMEINTERVALS))
  {
    queryCoreTimes = std::vector<seerep_core_msgs::Timeinterval>();
    for (auto&& timeinterval : *query->timeintervals())
    {
      seerep_core_msgs::Timeinterval queryCoreTime =
          seerep_core_msgs::Timeinterval();
      queryCoreTime.timeMin.seconds = timeinterval->time_min()->seconds();
      queryCoreTime.timeMax.seconds = timeinterval->time_max()->seconds();
      queryCoreTime.timeMin.nanos = timeinterval->time_min()->nanos();
      queryCoreTime.timeMax.nanos = timeinterval->time_max()->nanos();
      queryCoreTimes.value().push_back(queryCoreTime);
    }
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

std::optional<seerep_core_msgs::SparqlQuery>
CoreFbConversion::fromFbSparqlQuery(const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_SPARQLQUERY))
  {
    seerep_core_msgs::SparqlQuery sparql;
    sparql.category = query->sparqlQuery()->category()->str();
    sparql.sparql = query->sparqlQuery()->sparql()->str();
    sparql.variableNameOfConcept =
        query->sparqlQuery()->variableNameOfCategory()->str();
    return sparql;
  }

  return std::nullopt;
}

std::optional<std::string>
CoreFbConversion::fromFbOntologyURI(const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_ONTOLOGYURI))
  {
    return query->ontologyURI()->str();
  }

  return std::nullopt;
}

bool CoreFbConversion::fromFbQueryMustHaveAllLabels(
    const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query,
                                  seerep::fb::Query::VT_MUSTHAVEALLLABELS))
  {
    return query->mustHaveAllLabels();
  }

  return false;
}

bool CoreFbConversion::fromFbQueryInMapFrame(const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_INMAPFRAME))
  {
    return query->inMapFrame();
  }

  return false;
}

bool CoreFbConversion::fromFbQueryFullyEncapsulated(
    const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query,
                                  seerep::fb::Query::VT_FULLYENCAPSULATED))
  {
    return query->fullyEncapsulated();
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

void CoreFbConversion::fromFbDataHeader(const seerep::fb::Header* header,
                                        seerep_core_msgs::Header& coreHeader,
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
  coreHeader.sequence = header->seq();

  boost::uuids::string_generator gen;
  coreHeader.uuidProject = gen(header->uuid_project()->str());
}

boost::uuids::uuid
CoreFbConversion::fromFbDataHeaderUuid(const std::string& uuidMsg)
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

void CoreFbConversion::fromFbDataLabels(
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelCategory>>*
        labels,
    std::unordered_map<std::string, seerep_core_msgs::LabelDatumaro>&
        labelsPerCategory)
{
  if (labels)
  {
    for (auto labelsCategories : *labels)
    {
      seerep_core_msgs::LabelDatumaro* labelDatumaroPtr;

      auto catMap =
          labelsPerCategory.find(labelsCategories->category()->c_str());
      if (catMap != labelsPerCategory.end())
      {
        labelDatumaroPtr = &(catMap->second);
      }
      else
      {
        seerep_core_msgs::LabelDatumaro labelDatumaro;
        auto entry = labelsPerCategory.emplace(
            labelsCategories->category()->c_str(), labelDatumaro);
        labelDatumaroPtr = &(entry.first->second);
      }

      if (labelsCategories->labels())
      {
        seerep_core_fb::LabelVec labelVector;
        for (auto label : *labelsCategories->labels())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance = boost::uuids::nil_uuid();

          if (label->instanceUuid())
          {
            try
            {
              uuidInstance = gen(label->instanceUuid()->str());
            }
            catch (std::runtime_error const& e)
            {
              // is handled by the default value of uuidInstance
            }
          }

          labelVector.push_back(seerep_core_msgs::Label{
              .label = label->label()->str(),
              .labelIdDatumaro = label->labelIdDatumaro(),
              .uuidInstance = uuidInstance,
              .instanceIdDatumaro = label->instanceIdDatumaro() });
        }
        labelDatumaroPtr->datumaroJson =
            labelsCategories->datumaroJson()->str();
        labelDatumaroPtr->labels = labelVector;
      }
    }
  }
}

std::optional<seerep_core_msgs::Polygon2D>
CoreFbConversion::fromFbQueryPolygon(const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query, seerep::fb::Query::VT_POLYGON))
  {
    return extractPolygon(query->polygon()->vertices(),
                          query->polygon()->height(), query->polygon()->z());
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<seerep_core_msgs::Polygon2D>
CoreFbConversion::fromFbQueryPolygonSensorPosition(const seerep::fb::Query* query)
{
  if (flatbuffers::IsFieldPresent(query,
                                  seerep::fb::Query::VT_POLYGONSENSORPOSITION))
  {
    return extractPolygon(query->polygonSensorPosition()->vertices(),
                          query->polygonSensorPosition()->height(),
                          query->polygonSensorPosition()->z());
  }
  else
  {
    return std::nullopt;
  }
}

seerep_core_msgs::Polygon2D CoreFbConversion::extractPolygon(
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::Point2D>>* vertices,
    double height, double z)
{
  seerep_core_msgs::Polygon2D polygon;
  for (auto point : *vertices)
  {
    seerep_core_msgs::Point2D temp(point->x(), point->y());
    polygon.vertices.push_back(temp);
  }

  polygon.height = height;
  polygon.z = z;

  return polygon;
}

std::vector<seerep_core_msgs::Datatype>
CoreFbConversion::fromFbDatatypeVector(const seerep::fb::Datatype& dt)
{
  std::vector<seerep_core_msgs::Datatype> dt_vector;

  if (dt == seerep::fb::Datatype_All)
  {
    dt_vector.push_back(seerep_core_msgs::Datatype::Image);
    dt_vector.push_back(seerep_core_msgs::Datatype::PointCloud);
    dt_vector.push_back(seerep_core_msgs::Datatype::Point);
  }
  else
  {
    seerep_core_msgs::Datatype dtCore = fromFb(dt);
    dt_vector.push_back(dtCore);
  }

  return dt_vector;
}

}  // namespace seerep_core_fb
