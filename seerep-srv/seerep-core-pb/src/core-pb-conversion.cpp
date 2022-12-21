#include "seerep-core-pb/core-pb-conversion.h"

namespace seerep_core_pb
{
seerep_core_msgs::Query CorePbConversion::fromPb(const seerep::Query& query, seerep_core_msgs::Datatype datatype)
{
  seerep_core_msgs::Query queryCore;
  queryCore.header.datatype = datatype;

  fromPbBoundingBox(query, queryCore);
  fromPbTime(query, queryCore);
  fromPbLabel(query, queryCore);

  fromPbMustHaveAllLabels(query, queryCore);
  fromPbProject(query, queryCore);
  fromPbInstance(query, queryCore);
  fromPbDataUuids(query, queryCore);
  fromPbWithOutData(query, queryCore);
  fromFbQueryMaxNumData(query, queryCore);

  return queryCore;
}

seerep_core_msgs::DatasetIndexable CorePbConversion::fromPb(const seerep::Image& img)
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
  if (!img.general_labels().empty())
  {
    for (auto labelsCategories : img.general_labels())
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      if (!labelsCategories.label_with_instances().empty())
      {
        for (auto label : labelsCategories.label_with_instances())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.instance_uuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelWithInstanceVector.push_back(
              seerep_core_msgs::LabelWithInstance{ .label = label.label(), .uuidInstance = uuidInstance });
        }
        dataForIndices.labelsWithInstancesWithCategory.emplace(labelsCategories.category().c_str(),
                                                               labelWithInstanceVector);
      }
    }
  }

  if (!img.labeled_bounding_boxes().empty())
  {
    for (auto labelsCategories : img.labeled_bounding_boxes())
    {
      std::vector<seerep_core_msgs::LabelWithInstance> labelWithInstanceVector;
      if (!labelsCategories.labeled_2d_bounding_boxes().empty())
      {
        for (auto label : labelsCategories.labeled_2d_bounding_boxes())
        {
          boost::uuids::string_generator gen;
          boost::uuids::uuid uuidInstance;
          try
          {
            uuidInstance = gen(label.label_with_instance().instance_uuid());
          }
          catch (std::runtime_error const& e)
          {
            uuidInstance = boost::uuids::nil_uuid();
          }

          labelWithInstanceVector.push_back(seerep_core_msgs::LabelWithInstance{
              .label = label.label_with_instance().label(), .uuidInstance = uuidInstance });
        }
      }
      dataForIndices.labelsWithInstancesWithCategory.emplace(labelsCategories.category().c_str(),
                                                             labelWithInstanceVector);
    }
  }

  return dataForIndices;
}

seerep_core_msgs::QueryTf CorePbConversion::fromPb(const seerep::TransformStampedQuery& query)
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

void CorePbConversion::fromPbProject(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.project_uuids().empty())
  {
    queryCore.projects = std::vector<boost::uuids::uuid>();
    for (auto projectuuid : query.project_uuids())
    {
      queryCore.projects.value().push_back(gen(projectuuid));
    }
  }
}

void CorePbConversion::fromPbLabel(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (!query.categorized_labels().empty())
  {
    queryCore.label = std::unordered_map<std::string, std::vector<std::string>>();
    for (auto labelWithCategory : query.categorized_labels())
    {
      std::vector<std::string> labels;
      for (auto label : labelWithCategory.labels())
      {
        labels.push_back(label);
      }
      queryCore.label.value().emplace(labelWithCategory.category(), labels);
    }
  }
}

void CorePbConversion::fromPbTime(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.time_interval().has_time_min() && query.time_interval().has_time_max())
  {
    queryCore.timeinterval = seerep_core_msgs::Timeinterval();
    queryCore.timeinterval.value().timeMin.seconds = query.time_interval().time_min().seconds();
    queryCore.timeinterval.value().timeMax.seconds = query.time_interval().time_max().seconds();
    queryCore.timeinterval.value().timeMin.nanos = query.time_interval().time_min().nanos();
    queryCore.timeinterval.value().timeMax.nanos = query.time_interval().time_max().nanos();
  }
}

void CorePbConversion::fromPbBoundingBox(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  if (query.bounding_box_stamped().has_header() && query.bounding_box_stamped().has_bounding_box() &&
      query.bounding_box_stamped().bounding_box().has_point_min() &&
      query.bounding_box_stamped().bounding_box().has_point_max())
  {
    queryCore.header.frameId = query.bounding_box_stamped().header().frame_id();
    queryCore.boundingbox = seerep_core_msgs::AABB();
    queryCore.boundingbox.value().min_corner().set<0>(query.bounding_box_stamped().bounding_box().point_min().x());
    queryCore.boundingbox.value().min_corner().set<1>(query.bounding_box_stamped().bounding_box().point_min().y());
    queryCore.boundingbox.value().min_corner().set<2>(query.bounding_box_stamped().bounding_box().point_min().z());
    queryCore.boundingbox.value().max_corner().set<0>(query.bounding_box_stamped().bounding_box().point_max().x());
    queryCore.boundingbox.value().max_corner().set<1>(query.bounding_box_stamped().bounding_box().point_max().y());
    queryCore.boundingbox.value().max_corner().set<2>(query.bounding_box_stamped().bounding_box().point_max().z());
  }
}

void CorePbConversion::fromPbMustHaveAllLabels(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.mustHaveAllLabels = query.must_have_all_labels();
}

void CorePbConversion::fromPbInstance(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.instance_uuids().empty())
  {
    queryCore.instances = std::vector<boost::uuids::uuid>();
    for (auto instance : query.instance_uuids())
    {
      queryCore.instances.value().push_back(gen(instance));
    }
  }
}

void CorePbConversion::fromPbDataUuids(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  boost::uuids::string_generator gen;
  if (!query.data_uuids().empty())
  {
    queryCore.dataUuids = std::vector<boost::uuids::uuid>();
    for (auto data : query.data_uuids())
    {
      queryCore.dataUuids.value().push_back(gen(data));
    }
  }
}

void CorePbConversion::fromPbWithOutData(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.withoutData = query.without_data();
}

void CorePbConversion::fromFbQueryMaxNumData(const seerep::Query& query, seerep_core_msgs::Query& queryCore)
{
  queryCore.maxNumData = query.max_num_data();
}
}  // namespace seerep_core_pb
