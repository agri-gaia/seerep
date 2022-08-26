#include "seerep-hdf5-pb/hdf5-pb-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_pb
{
Hdf5PbGeneral::Hdf5PbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : seerep_hdf5_core::Hdf5CoreGeneral(file, write_mtx)
{
}

void Hdf5PbGeneral::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeled>& boundingboxLabeled)
{
  if (!boundingboxLabeled.empty())
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : boundingboxLabeled)
    {
      labels.push_back(label.labelwithinstance().label());
      std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                               label.boundingbox().point_min().z(), label.boundingbox().point_max().x(),
                               label.boundingbox().point_max().y(), label.boundingbox().point_max().z() };
      boundingBoxes.push_back(box);
      instances.push_back(label.labelwithinstance().instanceuuid());
    }

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES, HighFive::DataSpace::From(instances));
    datasetInstances.write(instances);

    m_file->flush();
  }
}

void Hdf5PbGeneral::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeled>& boundingbox2DLabeled)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!boundingbox2DLabeled.empty())
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : boundingbox2DLabeled)
    {
      labels.push_back(label.labelwithinstance().label());
      std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                               label.boundingbox().point_max().x(), label.boundingbox().point_max().y() };
      boundingBoxes.push_back(box);

      instances.push_back(label.labelwithinstance().instanceuuid());
    }

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
    datasetBoxes.write(boundingBoxes);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES, HighFive::DataSpace::From(instances));
    datasetInstances.write(instances);

    m_file->flush();
  }
}

std::optional<google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled>>
Hdf5PbGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  try
  {
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB);
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES);
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES);
  }
  catch (std::invalid_argument const& e)
  {
    return std::nullopt;
  }

  std::vector<std::string> labels;
  std::vector<std::vector<double>> boundingBoxes;
  std::vector<std::string> instances;

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB);
  datasetLabels.read(labels);

  HighFive::DataSet datasetBoxes = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES);
  datasetBoxes.read(boundingBoxes);

  HighFive::DataSet datasetInstances =
      m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES);
  datasetInstances.read(instances);

  if (labels.size() != boundingBoxes.size() || labels.size() != instances.size())
  {
    std::cout << "size of labels (" << labels.size() << "), size of bounding boxes (" << boundingBoxes.size()
              << ") and size of instances (" << instances.size() << ") do not fit." << std::endl;
    return std::nullopt;
  }

  google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeled> result;

  for (long unsigned int i = 0; i < labels.size(); i++)
  {
    seerep::BoundingBox2DLabeled bblabeled;
    bblabeled.mutable_labelwithinstance()->set_label(labels.at(i));
    bblabeled.mutable_labelwithinstance()->set_instanceuuid(instances.at(i));

    bblabeled.mutable_boundingbox()->mutable_point_min()->set_x(boundingBoxes.at(i).at(0));
    bblabeled.mutable_boundingbox()->mutable_point_min()->set_y(boundingBoxes.at(i).at(1));
    bblabeled.mutable_boundingbox()->mutable_point_max()->set_x(boundingBoxes.at(i).at(2));
    bblabeled.mutable_boundingbox()->mutable_point_max()->set_y(boundingBoxes.at(i).at(3));

    result.Add(std::move(bblabeled));
  }

  return result;
}

void Hdf5PbGeneral::writeLabelsGeneral(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<seerep::LabelWithInstance>& labelsGeneralWithInstances)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (!labelsGeneralWithInstances.empty())
  {
    std::vector<std::string> labels;
    std::vector<std::string> instances;
    for (auto labelWithInstances : labelsGeneralWithInstances)
    {
      labels.push_back(labelWithInstances.label());

      instances.push_back(labelWithInstances.instanceuuid());
    }

    HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL, HighFive::DataSpace::From(labels));
    datasetLabels.write(labels);

    HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
        id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES, HighFive::DataSpace::From(instances));
    datasetInstances.write(instances);

    m_file->flush();
  }
}

std::optional<google::protobuf::RepeatedPtrField<seerep::LabelWithInstance>>
Hdf5PbGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid)
{
  std::string id = datatypeGroup + "/" + uuid;
  try
  {
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL);
    checkExists(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES);
  }
  catch (std::invalid_argument const& e)
  {
    return std::nullopt;
  }

  std::vector<std::string> labels;
  std::vector<std::string> instances;

  HighFive::DataSet datasetLabels = m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL);
  datasetLabels.read(labels);

  HighFive::DataSet datasetInstances =
      m_file->getDataSet(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES);
  datasetInstances.read(instances);

  if (labels.size() != instances.size())
  {
    std::cout << "size of labels (" << labels.size() << ") and size of instances (" << instances.size()
              << ") do not fit." << std::endl;
    return std::nullopt;
  }

  google::protobuf::RepeatedPtrField<seerep::LabelWithInstance> result;

  for (long unsigned int i = 0; i < labels.size(); i++)
  {
    seerep::LabelWithInstance labelWithInstance;
    labelWithInstance.set_label(labels.at(i));
    labelWithInstance.set_instanceuuid(instances.at(i));
    result.Add(std::move(labelWithInstance));
  }

  return result;
}

} /* namespace seerep_hdf5_pb */
