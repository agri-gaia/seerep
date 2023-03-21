
namespace seerep_hdf5_py
{
template <int NumDimensions>
void Hdf5PyGeneral::writeBoundingBoxLabeled(const std::string& dataGroupId,
                                            const std::vector<CategorizedBoundingBoxLabel<NumDimensions>>& bbLabels)
{
  for (auto& categoryLabels : bbLabels)
  {
    if (!categoryLabels.labels_.empty())
    {
      std::vector<std::string> labels;
      std::vector<float> labelConfidences;
      std::vector<std::string> instanceUuids;
      std::vector<std::vector<double>> boundingBoxesWithRotation;

      labels.reserve(categoryLabels.labels_.size());
      instanceUuids.reserve(categoryLabels.labels_.size());
      boundingBoxesWithRotation.reserve(categoryLabels.labels_.size());

      for (auto& instanceLabel : categoryLabels.labels_)
      {
        labels.push_back(instanceLabel.label_.label_);
        labelConfidences.push_back(instanceLabel.label_.confidence_);
        instanceUuids.push_back(instanceLabel.label_.instanceUuid_);

        std::vector<double> boxWithRotation;
        boxWithRotation.reserve(2 * NumDimensions + instanceLabel.bbRotation_.size());

        for (int i = 0; i < NumDimensions; i++)
        {
          boxWithRotation.push_back(instanceLabel.bbCenter_[i]);
        }
        for (int i = 0; i < NumDimensions; i++)
        {
          boxWithRotation.push_back(instanceLabel.bbSpatialExtent_[i]);
        }
        for (std::size_t i = 0; i < instanceLabel.bbRotation_.size(); i++)
        {
          boxWithRotation.push_back(instanceLabel.bbRotation_[i]);
        }

        boundingBoxesWithRotation.push_back(boxWithRotation);
      }

      HighFive::DataSpace labelsDataSpace = HighFive::DataSpace::From(labels);
      auto datasetLabels = getHdf5DataSet<std::string>(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB +
                                                           "_" + categoryLabels.category_,
                                                       labelsDataSpace);
      datasetLabels->write(labels);

      HighFive::DataSpace labelConfidencesDataSpace = HighFive::DataSpace::From(labelConfidences);
      auto datasetLabelConfidences = getHdf5DataSet<float>(
          dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBCONFIDENCES + "_" + categoryLabels.category_,
          labelConfidencesDataSpace);
      datasetLabelConfidences->write(labelConfidences);

      HighFive::DataSpace instancesDataSpace = HighFive::DataSpace::From(instanceUuids);
      auto datasetInstances = getHdf5DataSet<std::string>(
          dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + categoryLabels.category_,
          instancesDataSpace);
      datasetInstances->write(instanceUuids);

      HighFive::DataSpace boxesDataSpace = HighFive::DataSpace::From(boundingBoxesWithRotation);
      auto datasetBoxes =
          getHdf5DataSet<double>(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXESWITHROTATION + "_" +
                                     categoryLabels.category_,
                                 boxesDataSpace);
      datasetBoxes->write(boundingBoxesWithRotation);
    }
  }
  m_file->flush();
}

template <int NumDimensions>
std::vector<CategorizedBoundingBoxLabel<NumDimensions>>
Hdf5PyGeneral::readBoundingBoxLabeled(const std::string& dataGroupId)
{
  auto hdf5GroupPtr = getHdf5Group(dataGroupId);

  std::vector<CategorizedBoundingBoxLabel<NumDimensions>> bbLabels;

  const std::vector<std::string> groupDatasets = hdf5GroupPtr->listObjectNames();
  for (const auto& datasetName : groupDatasets)
  {
    if (datasetName.rfind(seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES, 0) == 0)
    {
      std::string category = datasetName.substr(seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES.length() + 1);
      bbLabels.push_back(CategorizedBoundingBoxLabel<NumDimensions>(category));

      auto datasetLabels =
          getHdf5DataSet(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB + "_" + category);
      auto datasetLabelConfidences =
          getHdf5DataSet(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBCONFIDENCES + "_" + category);
      auto datasetInstances =
          getHdf5DataSet(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + category);
      auto datasetBoxes = getHdf5DataSet(dataGroupId + "/" +
                                         seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXESWITHROTATION + "_" + category);

      if (datasetLabels == nullptr || datasetLabelConfidences == nullptr || datasetInstances == nullptr ||
          datasetBoxes == nullptr)
      {
        throw std::invalid_argument("cannot find all bounding box label datasets for category " + category);
      }

      std::vector<std::string> labels;
      std::vector<float> labelConfidences;
      std::vector<std::string> instanceUuids;
      std::vector<std::vector<double>> boundingBoxesWithRotation;

      datasetLabels->read(labels);
      datasetLabelConfidences->read(labelConfidences);
      datasetInstances->read(instanceUuids);
      datasetBoxes->read(boundingBoxesWithRotation);

      if (labels.size() != instanceUuids.size() || labels.size() != labelConfidences.size() ||
          labels.size() != boundingBoxesWithRotation.size())
      {
        throw std::invalid_argument("amounts for labels, instance uuids and bounding boxes do not match for category " +
                                    category);
      }

      for (std::size_t i = 0; i < labels.size(); i++)
      {
        std::array<double, NumDimensions> bbCenter;
        std::array<double, NumDimensions> bbSpatialExtent;
        typename BoundingBoxLabel<NumDimensions>::RotationType bbRotation;

        if (boundingBoxesWithRotation[i].size() != 2 * NumDimensions + bbRotation.size())
        {
          throw std::invalid_argument(std::to_string(NumDimensions) +
                                      "-dimensional bounding box expected for category " + category + " but got " +
                                      std::to_string(boundingBoxesWithRotation[i].size() / 2 - bbRotation.size()) +
                                      "-dimensional one");
        }

        for (int j = 0; j < NumDimensions; j++)
        {
          bbCenter[j] = boundingBoxesWithRotation[i][j];
        }
        for (int j = 0; j < NumDimensions; j++)
        {
          bbSpatialExtent[j] = boundingBoxesWithRotation[i][NumDimensions + j];
        }
        for (std::size_t j = 0; j < bbRotation.size(); j++)
        {
          bbRotation[j] = boundingBoxesWithRotation[i][2 * NumDimensions + j];
        }

        InstanceLabel label(labels[i], labelConfidences[i], instanceUuids[i]);
        BoundingBoxLabel<NumDimensions> bbLabel(label, bbCenter, bbSpatialExtent, bbRotation);
        bbLabels[bbLabels.size() - 1].addLabel(bbLabel);
      }
    }
  }

  return bbLabels;
}

} /* namespace seerep_hdf5_py */
