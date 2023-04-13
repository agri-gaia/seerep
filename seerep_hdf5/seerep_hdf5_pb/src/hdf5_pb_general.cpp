#include "seerep_hdf5_pb/hdf5_pb_general.h"

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
        std::vector<float> labelConfidences;
        std::vector<std::vector<double>> boundingBoxesWithRotation;
        std::vector<std::string> instances;
        for (auto label : boundingboxLabeled.boundingboxlabeled())
        {
          labels.push_back(label.labelwithinstance().label().label());
          labelConfidences.push_back(label.labelwithinstance().label().confidence());
          std::vector<double> boxWithRotation{
            label.boundingbox().center_point().x(),   label.boundingbox().center_point().y(),
            label.boundingbox().center_point().z(),   label.boundingbox().spatial_extent().x(),
            label.boundingbox().spatial_extent().y(), label.boundingbox().spatial_extent().z(),
            label.boundingbox().rotation().x(),       label.boundingbox().rotation().y(),
            label.boundingbox().rotation().z(),       label.boundingbox().rotation().w()
          };
          boundingBoxesWithRotation.push_back(boxWithRotation);
          instances.push_back(label.labelwithinstance().instanceuuid());
        }

        HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + boundingboxLabeled.category(),
            HighFive::DataSpace::From(labels));
        datasetLabels.write(labels);

        HighFive::DataSet datasetLabelConfidences = m_file->createDataSet<float>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBCONFIDENCES + "_" + boundingboxLabeled.category(),
            HighFive::DataSpace::From(labelConfidences));
        datasetLabelConfidences.write(labelConfidences);

        HighFive::DataSet datasetBoxesWithRotation =
            m_file->createDataSet<double>(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXESWITHROTATION + "_" +
                                              boundingboxLabeled.category(),
                                          HighFive::DataSpace::From(boundingBoxesWithRotation));
        datasetBoxesWithRotation.write(boundingBoxesWithRotation);

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
        std::vector<float> labelConfidences;
        std::vector<std::vector<double>> boundingBoxesWithRotation;
        std::vector<std::string> instances;
        for (auto label : boundingbox2DLabeled.boundingbox2dlabeled())
        {
          labels.push_back(label.labelwithinstance().label().label());
          labelConfidences.push_back(label.labelwithinstance().label().confidence());
          std::vector<double> boxWithRotation{ label.boundingbox().center_point().x(),
                                               label.boundingbox().center_point().y(),
                                               label.boundingbox().spatial_extent().x(),
                                               label.boundingbox().spatial_extent().y(),
                                               label.boundingbox().rotation() };
          boundingBoxesWithRotation.push_back(boxWithRotation);

          instances.push_back(label.labelwithinstance().instanceuuid());
        }

        HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + boundingbox2DLabeled.category(),
            HighFive::DataSpace::From(labels));
        datasetLabels.write(labels);

        HighFive::DataSet datasetLabelConfidences = m_file->createDataSet<float>(
            id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBCONFIDENCES + "_" + boundingbox2DLabeled.category(),
            HighFive::DataSpace::From(labelConfidences));
        datasetLabelConfidences.write(labelConfidences);

        HighFive::DataSet datasetBoxesWithRotation =
            m_file->createDataSet<double>(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXESWITHROTATION + "_" +
                                              boundingbox2DLabeled.category(),
                                          HighFive::DataSpace::From(boundingBoxesWithRotation));
        datasetBoxesWithRotation.write(boundingBoxesWithRotation);

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
  std::vector<std::vector<float>> labelConfidencesPerCategory;
  std::vector<std::vector<std::vector<double>>> boundingBoxesPerCategory;
  std::vector<std::vector<std::string>> instancesPerCategory;

  seerep_hdf5_core::Hdf5CoreGeneral::readBoundingBoxLabeled(datatypeGroup, uuid, labelCategories, labelsPerCategory,
                                                            labelConfidencesPerCategory, boundingBoxesPerCategory,
                                                            instancesPerCategory);

  google::protobuf::RepeatedPtrField<seerep::pb::BoundingBox2DLabeledWithCategory> result;

  for (size_t iCategory = 0; iCategory < labelCategories.size(); iCategory++)
  {
    seerep::pb::BoundingBox2DLabeledWithCategory resultCat;
    resultCat.set_category(labelCategories.at(iCategory));
    for (size_t i = 0; i < labelsPerCategory.at(iCategory).size(); i++)
    {
      auto bblabeled = resultCat.add_boundingbox2dlabeled();
      bblabeled->mutable_labelwithinstance()->mutable_label()->set_label(labelsPerCategory.at(iCategory).at(i));
      bblabeled->mutable_labelwithinstance()->mutable_label()->set_confidence(
          labelConfidencesPerCategory.at(iCategory).at(i));
      bblabeled->mutable_labelwithinstance()->set_instanceuuid(instancesPerCategory.at(iCategory).at(i));

      bblabeled->mutable_boundingbox()->mutable_center_point()->set_x(
          boundingBoxesPerCategory.at(iCategory).at(i).at(0));
      bblabeled->mutable_boundingbox()->mutable_center_point()->set_y(
          boundingBoxesPerCategory.at(iCategory).at(i).at(1));
      bblabeled->mutable_boundingbox()->mutable_spatial_extent()->set_x(
          boundingBoxesPerCategory.at(iCategory).at(i).at(2));
      bblabeled->mutable_boundingbox()->mutable_spatial_extent()->set_y(
          boundingBoxesPerCategory.at(iCategory).at(i).at(3));
      bblabeled->mutable_boundingbox()->set_rotation(boundingBoxesPerCategory.at(iCategory).at(i).at(4));
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
      std::vector<float> labelConfidences;
      std::vector<std::string> instances;
      for (auto& labelWithInstances : labelsWithInstanceOfCategory.labelwithinstance())
      {
        labels.push_back(labelWithInstances.label().label());
        labelConfidences.push_back(labelWithInstances.label().confidence());

        instances.push_back(labelWithInstances.instanceuuid());
      }
      seerep_core_msgs::LabelsWithInstanceWithCategory labelsWithCategory;
      labelsWithCategory.category = labelsWithInstanceOfCategory.category();
      labelsWithCategory.labels = labels;
      labelsWithCategory.labelConfidences = labelConfidences;
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
      labelWithInstance->mutable_label()->set_label(labelsWithInstancesGeneralPerCategory.at(iCategory).at(i).label);
      labelWithInstance->mutable_label()->set_confidence(
          labelsWithInstancesGeneralPerCategory.at(iCategory).at(i).labelConfidence);
      labelWithInstance->set_instanceuuid(
          boost::lexical_cast<std::string>(labelsWithInstancesGeneralPerCategory.at(iCategory).at(i).uuidInstance));
    }
    result.Add(std::move(resultCat));
  }
  return result;
}

} /* namespace seerep_hdf5_pb */
