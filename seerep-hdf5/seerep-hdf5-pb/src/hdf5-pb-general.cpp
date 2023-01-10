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
    const google::protobuf::RepeatedPtrField<::seerep::pb::BoundingBoxLabeledWithCategory>&
        boundingboxLabeledWithCategory)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!boundingboxLabeledWithCategory.empty())
  {
    for (auto& boundingboxLabeled : boundingboxLabeledWithCategory)
    {
      if (!boundingboxLabeled.boundingboxlabeled().empty())
      {
        std::vector<std::string> labels;
        std::vector<std::vector<double>> boundingBoxes;
        std::vector<std::string> instances;
        for (auto label : boundingboxLabeled.boundingboxlabeled())
        {
          labels.push_back(label.labelwithinstance().label());
          std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                                   label.boundingbox().point_min().z(), label.boundingbox().point_max().x(),
                                   label.boundingbox().point_max().y(), label.boundingbox().point_max().z() };
          boundingBoxes.push_back(box);
          instances.push_back(label.labelwithinstance().instanceuuid());
        }

        HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + boundingboxLabeled.category(),
            HighFive::DataSpace::From(labels));
        datasetLabels.write(labels);

        HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES + "_" + boundingboxLabeled.category(),
            HighFive::DataSpace::From(boundingBoxes));
        datasetBoxes.write(boundingBoxes);

        HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + boundingboxLabeled.category(),
            HighFive::DataSpace::From(instances));
        datasetInstances.write(instances);
      }
    }
    m_file->flush();
  }
}

void Hdf5PbGeneral::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<::seerep::pb::BoundingBox2DLabeledWithCategory>&
        boundingbox2DLabeledWithCategory)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!boundingbox2DLabeledWithCategory.empty())
  {
    for (auto& boundingbox2DLabeled : boundingbox2DLabeledWithCategory)
    {
      if (!boundingbox2DLabeled.boundingbox2dlabeled().empty())
      {
        std::vector<std::string> labels;
        std::vector<std::vector<double>> boundingBoxes;
        std::vector<std::string> instances;
        for (auto label : boundingbox2DLabeled.boundingbox2dlabeled())
        {
          labels.push_back(label.labelwithinstance().label());
          std::vector<double> box{ label.boundingbox().point_min().x(), label.boundingbox().point_min().y(),
                                   label.boundingbox().point_max().x(), label.boundingbox().point_max().y() };
          boundingBoxes.push_back(box);

          instances.push_back(label.labelwithinstance().instanceuuid());
        }

        HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + boundingbox2DLabeled.category(),
            HighFive::DataSpace::From(labels));
        datasetLabels.write(labels);

        HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES + "_" + boundingbox2DLabeled.category(),
            HighFive::DataSpace::From(boundingBoxes));
        datasetBoxes.write(boundingBoxes);

        HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + boundingbox2DLabeled.category(),
            HighFive::DataSpace::From(instances));
        datasetInstances.write(instances);

        m_file->flush();
      }
    }
  }
}

std::optional<google::protobuf::RepeatedPtrField<seerep::pb::BoundingBox2DLabeledWithCategory>>
Hdf5PbGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid)
{
  std::vector<std::string> labelCategories;
  std::vector<std::vector<std::string>> labelsPerCategory;
  std::vector<std::vector<std::vector<double>>> boundingBoxesPerCategory;
  std::vector<std::vector<std::string>> instancesPerCategory;

  seerep_hdf5_core::Hdf5CoreGeneral::readBoundingBoxLabeled(datatypeGroup, uuid, labelCategories, labelsPerCategory,
                                                            boundingBoxesPerCategory, instancesPerCategory);

  google::protobuf::RepeatedPtrField<seerep::pb::BoundingBox2DLabeledWithCategory> result;

  for (size_t iCategory = 0; iCategory < labelCategories.size(); iCategory++)
  {
    seerep::pb::BoundingBox2DLabeledWithCategory resultCat;
    resultCat.set_category(labelCategories.at(iCategory));
    for (size_t i = 0; i < labelsPerCategory.at(iCategory).size(); i++)
    {
      auto bblabeled = resultCat.add_boundingbox2dlabeled();
      bblabeled->mutable_labelwithinstance()->set_label(labelsPerCategory.at(iCategory).at(i));
      bblabeled->mutable_labelwithinstance()->set_instanceuuid(instancesPerCategory.at(iCategory).at(i));

      bblabeled->mutable_boundingbox()->mutable_point_min()->set_x(boundingBoxesPerCategory.at(iCategory).at(i).at(0));
      bblabeled->mutable_boundingbox()->mutable_point_min()->set_y(boundingBoxesPerCategory.at(iCategory).at(i).at(1));
      bblabeled->mutable_boundingbox()->mutable_point_max()->set_x(boundingBoxesPerCategory.at(iCategory).at(i).at(2));
      bblabeled->mutable_boundingbox()->mutable_point_max()->set_y(boundingBoxesPerCategory.at(iCategory).at(i).at(3));
    }
    result.Add(std::move(resultCat));
  }

  return result;
}

void Hdf5PbGeneral::writeLabelsGeneral(
    const std::string& datatypeGroup, const std::string& uuid,
    const google::protobuf::RepeatedPtrField<seerep::pb::LabelsWithInstanceWithCategory>&
        labelsGeneralWithInstancesWithCategory)
{
  if (!labelsGeneralWithInstancesWithCategory.empty())
  {
    std::vector<seerep_core_msgs::LabelsWithInstanceWithCategory> labelsWithCategoryVector;
    for (auto& labelsWithInstanceOfCategory : labelsGeneralWithInstancesWithCategory)
    {
      std::vector<std::string> labels;
      std::vector<std::string> instances;
      for (auto& labelWithInstances : labelsWithInstanceOfCategory.labelwithinstance())
      {
        labels.push_back(labelWithInstances.label());

        instances.push_back(labelWithInstances.instanceuuid());
      }
      seerep_core_msgs::LabelsWithInstanceWithCategory labelsWithCategory;
      labelsWithCategory.category = labelsWithInstanceOfCategory.category();
      labelsWithCategory.labels = labels;
      labelsWithCategory.instances = instances;
      labelsWithCategoryVector.push_back(labelsWithCategory);
    }
    Hdf5CoreGeneral::writeLabelsGeneral(datatypeGroup, uuid, labelsWithCategoryVector);
  }
}

std::optional<google::protobuf::RepeatedPtrField<seerep::pb::LabelsWithInstanceWithCategory>>
Hdf5PbGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid)
{
  std::vector<std::string> labelCategoriesGeneral;
  std::vector<std::vector<seerep_core_msgs::LabelWithInstance>> labelsWithInstancesGeneralPerCategory;
  seerep_hdf5_core::Hdf5CoreGeneral::readLabelsGeneral(datatypeGroup, uuid, labelCategoriesGeneral,
                                                       labelsWithInstancesGeneralPerCategory);

  google::protobuf::RepeatedPtrField<seerep::pb::LabelsWithInstanceWithCategory> result;

  for (size_t iCategory = 0; iCategory < labelCategoriesGeneral.size(); iCategory++)
  {
    seerep::pb::LabelsWithInstanceWithCategory resultCat;
    resultCat.set_category(labelCategoriesGeneral.at(iCategory));
    for (size_t i = 0; i < labelsWithInstancesGeneralPerCategory.at(iCategory).size(); i++)
    {
      auto labelWithInstance = resultCat.add_labelwithinstance();
      labelWithInstance->set_label(labelsWithInstancesGeneralPerCategory.at(iCategory).at(i).label);
      labelWithInstance->set_instanceuuid(
          boost::lexical_cast<std::string>(labelsWithInstancesGeneralPerCategory.at(iCategory).at(i).uuidInstance));
    }
    result.Add(std::move(resultCat));
  }
  return result;
}

} /* namespace seerep_hdf5_pb */
