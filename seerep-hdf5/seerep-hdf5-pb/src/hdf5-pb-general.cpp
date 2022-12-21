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
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBoxLabeledWithCategory>& boundingboxLabeledWithCategory)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!boundingboxLabeledWithCategory.empty())
  {
    for (auto& boundingboxLabeled : boundingboxLabeledWithCategory)
    {
      if (!boundingboxLabeled.labeled_bounding_boxes().empty())
      {
        std::vector<std::string> labels;
        std::vector<std::vector<double>> boundingBoxes;
        std::vector<std::string> instances;
        for (auto label : boundingboxLabeled.labeled_bounding_boxes())
        {
          labels.push_back(label.label_with_instance().label());
          std::vector<double> box{ label.bounding_box().point_min().x(), label.bounding_box().point_min().y(),
                                   label.bounding_box().point_min().z(), label.bounding_box().point_max().x(),
                                   label.bounding_box().point_max().y(), label.bounding_box().point_max().z() };
          boundingBoxes.push_back(box);
          instances.push_back(label.label_with_instance().instance_uuid());
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
    const google::protobuf::RepeatedPtrField<::seerep::BoundingBox2DLabeledWithCategory>&
        boundingbox2DLabeledWithCategory)
{
  std::string id = datatypeGroup + "/" + uuid;
  if (!boundingbox2DLabeledWithCategory.empty())
  {
    for (auto& boundingbox2DLabeled : boundingbox2DLabeledWithCategory)
    {
      if (!boundingbox2DLabeled.labeled_2d_bounding_boxes().empty())
      {
        std::vector<std::string> labels;
        std::vector<std::vector<double>> boundingBoxes;
        std::vector<std::string> instances;
        for (auto label : boundingbox2DLabeled.labeled_2d_bounding_boxes())
        {
          labels.push_back(label.label_with_instance().label());
          std::vector<double> box{ label.bounding_box().point_min().x(), label.bounding_box().point_min().y(),
                                   label.bounding_box().point_max().x(), label.bounding_box().point_max().y() };
          boundingBoxes.push_back(box);

          instances.push_back(label.label_with_instance().instance_uuid());
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

std::optional<google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeledWithCategory>>
Hdf5PbGeneral::readBoundingBox2DLabeled(const std::string& datatypeGroup, const std::string& uuid)
{
  std::vector<std::string> labelCategories;
  std::vector<std::vector<std::string>> labelsPerCategory;
  std::vector<std::vector<std::vector<double>>> boundingBoxesPerCategory;
  std::vector<std::vector<std::string>> instancesPerCategory;

  seerep_hdf5_core::Hdf5CoreGeneral::readBoundingBoxLabeled(datatypeGroup, uuid, labelCategories, labelsPerCategory,
                                                            boundingBoxesPerCategory, instancesPerCategory);

  google::protobuf::RepeatedPtrField<seerep::BoundingBox2DLabeledWithCategory> result;

  for (size_t iCategory = 0; iCategory < labelCategories.size(); iCategory++)
  {
    seerep::BoundingBox2DLabeledWithCategory resultCat;
    resultCat.set_category(labelCategories.at(iCategory));
    for (size_t i = 0; i < labelsPerCategory.at(iCategory).size(); i++)
    {
      auto bblabeled = resultCat.add_labeled_2d_bounding_boxes();
      bblabeled->mutable_label_with_instance()->set_label(labelsPerCategory.at(iCategory).at(i));
      bblabeled->mutable_label_with_instance()->set_instance_uuid(instancesPerCategory.at(iCategory).at(i));

      bblabeled->mutable_bounding_box()->mutable_point_min()->set_x(boundingBoxesPerCategory.at(iCategory).at(i).at(0));
      bblabeled->mutable_bounding_box()->mutable_point_min()->set_y(boundingBoxesPerCategory.at(iCategory).at(i).at(1));
      bblabeled->mutable_bounding_box()->mutable_point_max()->set_x(boundingBoxesPerCategory.at(iCategory).at(i).at(2));
      bblabeled->mutable_bounding_box()->mutable_point_max()->set_y(boundingBoxesPerCategory.at(iCategory).at(i).at(3));
    }
    result.Add(std::move(resultCat));
  }

  return result;
}

void Hdf5PbGeneral::writeLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid,
                                       const google::protobuf::RepeatedPtrField<seerep::LabelsWithInstanceWithCategory>&
                                           labelsGeneralWithInstancesWithCategory)
{
  if (!labelsGeneralWithInstancesWithCategory.empty())
  {
    std::vector<seerep_core_msgs::LabelsWithInstanceWithCategory> labelsWithCategoryVector;
    for (auto& labelsWithInstanceOfCategory : labelsGeneralWithInstancesWithCategory)
    {
      std::vector<std::string> labels;
      std::vector<std::string> instances;
      for (auto& labelWithInstances : labelsWithInstanceOfCategory.label_with_instances())
      {
        labels.push_back(labelWithInstances.label());

        instances.push_back(labelWithInstances.instance_uuid());
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

std::optional<google::protobuf::RepeatedPtrField<seerep::LabelsWithInstanceWithCategory>>
Hdf5PbGeneral::readLabelsGeneral(const std::string& datatypeGroup, const std::string& uuid)
{
  std::vector<std::string> labelCategoriesGeneral;
  std::vector<std::vector<seerep_core_msgs::LabelWithInstance>> labelsWithInstancesGeneralPerCategory;
  seerep_hdf5_core::Hdf5CoreGeneral::readLabelsGeneral(datatypeGroup, uuid, labelCategoriesGeneral,
                                                       labelsWithInstancesGeneralPerCategory);

  google::protobuf::RepeatedPtrField<seerep::LabelsWithInstanceWithCategory> result;

  for (size_t iCategory = 0; iCategory < labelCategoriesGeneral.size(); iCategory++)
  {
    seerep::LabelsWithInstanceWithCategory resultCat;
    resultCat.set_category(labelCategoriesGeneral.at(iCategory));
    for (size_t i = 0; i < labelsWithInstancesGeneralPerCategory.at(iCategory).size(); i++)
    {
      auto labelWithInstance = resultCat.add_label_with_instances();
      labelWithInstance->set_label(labelsWithInstancesGeneralPerCategory.at(iCategory).at(i).label);
      labelWithInstance->set_instance_uuid(
          boost::lexical_cast<std::string>(labelsWithInstancesGeneralPerCategory.at(iCategory).at(i).uuidInstance));
    }
    result.Add(std::move(resultCat));
  }
  return result;
}

} /* namespace seerep_hdf5_pb */
