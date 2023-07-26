#include "seerep_core_pb/core_pb_conversion.h"

namespace seerep_core_pb
{
seerep_core_msgs::Query CorePbConversion::fromPb(const seerep::pb::Query& query, seerep_core_msgs::Datatype datatype)
{
  seerep_core_msgs::Query queryCore;
  queryCore.header.datatype = datatype;

  fromPbBoundingBox(query, queryCore);
  fromPbPolygon(query, queryCore);
  fromPbTime(query, queryCore);
  fromPbLabel(query, queryCore);

  fromPbMustHaveAllLabels(query, queryCore);
  fromPbProject(query, queryCore);
  fromPbInstance(query, queryCore);
  fromPbDataUuids(query, queryCore);
  fromPbWithOutData(query, queryCore);
  fromFbQueryMaxNumData(query, queryCore);
  fromPbFullyEncapsulated(query, queryCore);

  return queryCore;
}

seerep_core_msgs::DatasetIndexable CorePbConversion::fromPb(const seerep::pb::Image& img)
{
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid;
  if (img.header().uuid_msgs().empty())
  {
    uuid = boost::uuids::random_generator()();
  }
  else
  {
    uuid = gen(img.header().uuid_msgs());
  }

  seerep_core_msgs::DatasetIndexable dataForIndices;
  dataForIndices.header.datatype = seerep_core_msgs::Datatype::Image;
  dataForIndices.header.frameId = img.header().frame_id();
  dataForIndices.header.timestamp.seconds = img.header().stamp().seconds();
  dataForIndices.header.timestamp.nanos = img.header().stamp().nanos();
  dataForIndices.header.uuidData = uuid;
  dataForIndices.header.uuidProject = gen(img.header().uuid_project());
  // set bounding box for images to 0. assume no spatial extent
  dataForIndices.boundingbox.min_corner().set<0>(0);
  dataForIndices.boundingbox.min_corner().set<1>(0);
  dataForIndices.boundingbox.min_corner().set<2>(0);
  dataForIndices.boundingbox.max_corner().set<0>(0);
  dataForIndices.boundingbox.max_corner().set<1>(0);
  dataForIndices.boundingbox.max_corner().set<2>(0);

  // semantic
  if (!img.labels_general().empty())
  {
    for (auto labelsCategories : img.labels_general())
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      if (!labelsCategories.labelwithinstance().empty())
      {
        for (auto label : labelsCategories.labelwithinstance())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.instanceuuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelWithInstanceVector.push_back(
              seerep_core_msgs::LabelWithInstance{ .label = label.label().label(),
                                                   .labelConfidence = label.label().confidence(),
                                                   .uuidInstance = uuidInstance });
        }
        dataForIndices.labelsWithInstancesWithCategory.emplace(labelsCategories.category().c_str(),
                                                               labelWithInstanceVector);
      }
    }
  }

  if (!img.labels_bb().empty())
  {
    for (auto labelsCategories : img.labels_bb())
    {
      std::unique_ptr<LabelWithInstanceVec> labelWithInstanceVecPtr;

      auto catMap = dataForIndices.labelsWithInstancesWithCategory.find(labelsCategories.category().c_str());
      if (catMap != dataForIndices.labelsWithInstancesWithCategory.end())
      {
        labelWithInstanceVecPtr = std::make_unique<LabelWithInstanceVec>(catMap->second);
      }
      else
      {
        std::vector<seerep_core_msgs::LabelWithInstance> labelVector;
        auto entry =
            dataForIndices.labelsWithInstancesWithCategory.emplace(labelsCategories.category().c_str(), labelVector);
        labelWithInstanceVecPtr = std::make_unique<LabelWithInstanceVec>(entry.first->second);
      }

      if (!labelsCategories.boundingbox2dlabeled().empty())
      {
        for (auto label : labelsCategories.boundingbox2dlabeled())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.labelwithinstance().instanceuuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelWithInstanceVecPtr->push_back(
              seerep_core_msgs::LabelWithInstance{ .label = label.labelwithinstance().label().label(),
                                                   .labelConfidence = label.labelwithinstance().label().confidence(),
                                                   .uuidInstance = uuidInstance });
        }
      }
    }
  }

  return dataForIndices;
}

seerep_core_msgs::QueryTf CorePbConversion::fromPb(const seerep::pb::TransformStampedQuery& query)
{
  boost::uuids::string_generator gen;
  seerep_core_msgs::QueryTf queryTf;
  queryTf.childFrameId = query.child_frame_id();
  queryTf.parentFrameId = query.header().frame_id();
  queryTf.project = gen(query.header().uuid_project());
  queryTf.timestamp.seconds = query.header().stamp().seconds();
  queryTf.timestamp.nanos = query.header().stamp().nanos();

  return queryTf;
}

seerep_core_msgs::region_of_interest CorePbConversion::fromPb(const seerep::pb::RegionOfInterest& roi)
{
  seerep_core_msgs::region_of_interest roi_core;
  roi_core.height = roi.height();
  roi_core.width = roi.width();
  roi_core.x_offset = roi.x_offset();
  roi_core.y_offset = roi.y_offset();
  roi_core.do_rectify = roi.do_rectify();

  return roi_core;
}

seerep::pb::RegionOfInterest CorePbConversion::toPb(const seerep_core_msgs::region_of_interest& roi_core)
{
  seerep::pb::RegionOfInterest roi;
  roi.set_height(roi_core.height);
  roi.set_width(roi_core.width);
  roi.set_x_offset(roi_core.x_offset);
  roi.set_y_offset(roi_core.y_offset);
  roi.set_do_rectify(roi_core.do_rectify);

  return roi;
}

seerep_core_msgs::Timestamp CorePbConversion::fromPb(const seerep::pb::Timestamp& timestamp)
{
  seerep_core_msgs::Timestamp ts_core;
  ts_core.nanos = timestamp.nanos();
  ts_core.seconds = timestamp.seconds();

  return ts_core;
}

seerep::pb::Timestamp CorePbConversion::toPb(const seerep_core_msgs::Timestamp& timestamp)
{
  seerep::pb::Timestamp ts_pb;
  ts_pb.set_nanos(timestamp.nanos);
  ts_pb.set_seconds(timestamp.seconds);

  return ts_pb;
}

seerep_core_msgs::Header CorePbConversion::fromPb(const seerep::pb::Header& header)
{
  seerep_core_msgs::Header header_core;
  header_core.frameId = header.frame_id();
  header_core.sequence = header.seq();
  header_core.timestamp = CorePbConversion::fromPb(header.stamp());
  header_core.uuidData = boost::lexical_cast<boost::uuids::uuid>(header.uuid_msgs());
  header_core.uuidProject = boost::lexical_cast<boost::uuids::uuid>(header.uuid_project());

  return header_core;
}

seerep::pb::Header CorePbConversion::toPb(const seerep_core_msgs::Header& header)
{
  seerep::pb::Header header_pb;

  *header_pb.mutable_frame_id() = header.frameId;

  header_pb.set_seq(header.sequence);

  seerep::pb::Timestamp ts_pb = CorePbConversion::toPb(header.timestamp);
  *header_pb.mutable_stamp() = ts_pb;

  header_pb.set_uuid_msgs(boost::lexical_cast<std::string>(header.uuidData));
  header_pb.set_uuid_project(boost::lexical_cast<std::string>(header.uuidProject));

  return header_pb;
}

seerep_core_msgs::camera_intrinsics_query
CorePbConversion::fromPb(const seerep::pb::CameraIntrinsicsQuery& camintrinsics_query)
{
  seerep_core_msgs::camera_intrinsics_query camintrinsics_query_core;
  camintrinsics_query_core.uuidCameraIntrinsics =
      boost::lexical_cast<boost::uuids::uuid>(camintrinsics_query.uuid_camera_intrinsics());
  camintrinsics_query_core.uuidProject = boost::lexical_cast<boost::uuids::uuid>(camintrinsics_query.uuid_project());

  return camintrinsics_query_core;
}

seerep_core_msgs::camera_intrinsics CorePbConversion::fromPb(const seerep::pb::CameraIntrinsics& camintrinsics)
{
  seerep_core_msgs::camera_intrinsics camintrinsics_core;
  camintrinsics_core.binning_x = camintrinsics.binning_x();
  camintrinsics_core.binning_y = camintrinsics.binning_y();

  for (double d : camintrinsics.distortion())
  {
    camintrinsics_core.distortion.push_back(d);
  }

  camintrinsics_core.distortion_model = camintrinsics.distortion_model();
  camintrinsics_core.header = CorePbConversion::fromPb(camintrinsics.header());
  camintrinsics_core.height = camintrinsics.height();
  camintrinsics_core.width = camintrinsics.width();

  for (double im : camintrinsics.intrinsic_matrix())
  {
    camintrinsics_core.intrinsic_matrix.push_back(im);
  }

  for (double pm : camintrinsics.projection_matrix())
  {
    camintrinsics_core.projection_matrix.push_back(pm);
  }

  for (double rm : camintrinsics.rectification_matrix())
  {
    camintrinsics_core.rectification_matrix.push_back(rm);
  }

  camintrinsics_core.region_of_interest = CorePbConversion::fromPb(camintrinsics.region_of_interest());

  camintrinsics_core.maximum_viewing_distance = camintrinsics.maximum_viewing_distance();

  return camintrinsics_core;
}

seerep::pb::CameraIntrinsics CorePbConversion::toPb(const seerep_core_msgs::camera_intrinsics& camintrinsics)
{
  seerep::pb::CameraIntrinsics ci_pb;
  ci_pb.set_binning_x(camintrinsics.binning_x);
  ci_pb.set_binning_y(camintrinsics.binning_y);

  for (auto d : camintrinsics.distortion)
  {
    ci_pb.add_distortion(d);
  }

  std::string dm = camintrinsics.distortion_model;
  ci_pb.set_distortion_model(dm);

  seerep::pb::Header header_pb = CorePbConversion::toPb(camintrinsics.header);
  *ci_pb.mutable_header() = header_pb;

  ci_pb.set_height(camintrinsics.height);
  ci_pb.set_width(camintrinsics.width);

  for (auto im : camintrinsics.intrinsic_matrix)
  {
    ci_pb.add_intrinsic_matrix(im);
  }

  for (auto pm : camintrinsics.projection_matrix)
  {
    ci_pb.add_projection_matrix(pm);
  }

  for (auto rm : camintrinsics.rectification_matrix)
  {
    ci_pb.add_projection_matrix(rm);
  }

  seerep::pb::RegionOfInterest roi_pb = CorePbConversion::toPb(camintrinsics.region_of_interest);
  *ci_pb.mutable_region_of_interest() = roi_pb;

  ci_pb.set_maximum_viewing_distance(camintrinsics.maximum_viewing_distance);

  return ci_pb;
}

void CorePbConversion::fromPbProject(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.projectuuid().empty())
  {
    queryCore.projects = std::vector<boost::uuids::uuid>();
    for (auto projectuuid : query.projectuuid())
    {
      queryCore.projects.value().push_back(gen(projectuuid));
    }
  }
}

void CorePbConversion::fromPbLabel(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (!query.labelswithcategory().empty())
  {
    queryCore.label = std::unordered_map<std::string, std::vector<std::string>>();
    for (auto labelWithCategory : query.labelswithcategory())
    {
      std::vector<std::string> labels;
      for (auto label : labelWithCategory.labels())
      {
        labels.push_back(label.label());
      }
      queryCore.label.value().emplace(labelWithCategory.category(), labels);
    }
  }
}

void CorePbConversion::fromPbTime(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.timeinterval().has_time_min() && query.timeinterval().has_time_max())
  {
    queryCore.timeinterval = seerep_core_msgs::Timeinterval();
    queryCore.timeinterval.value().timeMin.seconds = query.timeinterval().time_min().seconds();
    queryCore.timeinterval.value().timeMax.seconds = query.timeinterval().time_max().seconds();
    queryCore.timeinterval.value().timeMin.nanos = query.timeinterval().time_min().nanos();
    queryCore.timeinterval.value().timeMax.nanos = query.timeinterval().time_max().nanos();
  }
}

seerep_core_msgs::Point2D CorePbConversion::fromPbPoint2D(const seerep::pb::Point2D& point)
{
  seerep_core_msgs::Point2D pointCore;

  pointCore.set<0>(point.x());
  pointCore.set<1>(point.y());

  return pointCore;
}

void CorePbConversion::fromPbPolygon(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.has_polygon())
  {
    queryCore.polygon = seerep_core_msgs::Polygon2D();

    queryCore.polygon->height = query.polygon().height();
    queryCore.polygon->z = query.polygon().z();
    for (auto vertex : query.polygon().vertices())
    {
      seerep_core_msgs::Point2D p = fromPbPoint2D(vertex);
      queryCore.polygon->vertices.push_back(p);
    }
  }
}

void CorePbConversion::fromPbBoundingBox(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.boundingboxstamped().has_header() && query.boundingboxstamped().has_boundingbox() &&
      query.boundingboxstamped().boundingbox().has_center_point() &&
      query.boundingboxstamped().boundingbox().has_spatial_extent())
  {
    queryCore.header.frameId = query.boundingboxstamped().header().frame_id();
    queryCore.boundingbox = seerep_core_msgs::AABB();
    queryCore.boundingbox.value().min_corner().set<0>(query.boundingboxstamped().boundingbox().center_point().x() -
                                                      query.boundingboxstamped().boundingbox().spatial_extent().x() /
                                                          2.0);
    queryCore.boundingbox.value().min_corner().set<1>(query.boundingboxstamped().boundingbox().center_point().y() -
                                                      query.boundingboxstamped().boundingbox().spatial_extent().y() /
                                                          2.0);
    queryCore.boundingbox.value().min_corner().set<2>(query.boundingboxstamped().boundingbox().center_point().z() -
                                                      query.boundingboxstamped().boundingbox().spatial_extent().z() /
                                                          2.0);
    queryCore.boundingbox.value().max_corner().set<0>(query.boundingboxstamped().boundingbox().center_point().x() +
                                                      query.boundingboxstamped().boundingbox().spatial_extent().x() /
                                                          2.0);
    queryCore.boundingbox.value().max_corner().set<1>(query.boundingboxstamped().boundingbox().center_point().y() +
                                                      query.boundingboxstamped().boundingbox().spatial_extent().y() /
                                                          2.0);
    queryCore.boundingbox.value().max_corner().set<2>(query.boundingboxstamped().boundingbox().center_point().z() +
                                                      query.boundingboxstamped().boundingbox().spatial_extent().z() /
                                                          2.0);
  }
}

void CorePbConversion::fromPbMustHaveAllLabels(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.mustHaveAllLabels = query.musthavealllabels();
}

void CorePbConversion::fromPbFullyEncapsulated(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.fullyEncapsulated = query.fullyencapsulated();
}

void CorePbConversion::fromPbInstance(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.instanceuuid().empty())
  {
    queryCore.instances = std::vector<boost::uuids::uuid>();
    for (auto instance : query.instanceuuid())
    {
      queryCore.instances.value().push_back(gen(instance));
    }
  }
}

void CorePbConversion::fromPbDataUuids(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.datauuid().empty())
  {
    queryCore.dataUuids = std::vector<boost::uuids::uuid>();
    for (auto data : query.datauuid())
    {
      queryCore.dataUuids.value().push_back(gen(data));
    }
  }
}

void CorePbConversion::fromPbWithOutData(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.withoutData = query.withoutdata();
}

void CorePbConversion::fromFbQueryMaxNumData(const seerep::pb::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.maxNumData = query.maxnumdata();
}

void CorePbConversion::fromPbDatatypeVector(const seerep::datatype& datatype,
                                            std::vector<seerep_core_msgs::Datatype>& dtCore)
{
  if (datatype == seerep::datatype::image)
  {
    dtCore.push_back(seerep_core_msgs::Datatype::Image);
  }
  else if (datatype == seerep::datatype::pointcloud)
  {
    dtCore.push_back(seerep_core_msgs::Datatype::PointCloud);
  }
  else if (datatype == seerep::datatype::point)
  {
    dtCore.push_back(seerep_core_msgs::Datatype::Point);
  }
  else if (datatype == seerep::datatype::all)
  {
    dtCore.push_back(seerep_core_msgs::Datatype::Image);
    dtCore.push_back(seerep_core_msgs::Datatype::PointCloud);
    dtCore.push_back(seerep_core_msgs::Datatype::Point);
  }
}

void CorePbConversion::toPb(const seerep_core_msgs::AabbTime& timeinterval, seerep::pb::TimeInterval* ti_pb)
{
  // isolate second and nano second bits from min time
  int64_t mintime = timeinterval.min_corner().get<0>();
  int32_t min_nanos = (int32_t)mintime;
  uint32_t min_seconds = (uint32_t)(mintime >> 32);

  // isolate second and nano second bits from max time
  int64_t maxtime = timeinterval.max_corner().get<0>();
  int32_t max_nanos = (int32_t)maxtime;
  uint32_t max_seconds = (uint32_t)(maxtime >> 32);

  ti_pb->mutable_time_min()->set_nanos(min_nanos);
  ti_pb->mutable_time_min()->set_seconds(min_seconds);

  ti_pb->mutable_time_max()->set_nanos(max_nanos);
  ti_pb->mutable_time_max()->set_seconds(max_seconds);
}

void CorePbConversion::toPb(const seerep_core_msgs::AABB& aabb, seerep::pb::Boundingbox* bb_pb)
{
  // center
  float center_x = (aabb.min_corner().get<0>() + aabb.max_corner().get<0>()) / 2;
  float center_y = (aabb.min_corner().get<1>() + aabb.max_corner().get<1>()) / 2;
  float center_z = (aabb.min_corner().get<2>() + aabb.max_corner().get<2>()) / 2;

  // spatial extent
  float se_x = (aabb.max_corner().get<0>() - aabb.min_corner().get<0>());
  float se_y = (aabb.max_corner().get<1>() - aabb.min_corner().get<1>());
  float se_z = (aabb.max_corner().get<2>() - aabb.min_corner().get<2>());

  bb_pb->mutable_center_point()->set_x(center_x);
  bb_pb->mutable_center_point()->set_y(center_y);
  bb_pb->mutable_center_point()->set_z(center_z);

  bb_pb->mutable_spatial_extent()->set_x(se_x);
  bb_pb->mutable_spatial_extent()->set_y(se_y);
  bb_pb->mutable_spatial_extent()->set_z(se_z);
}

}  // namespace seerep_core_pb
