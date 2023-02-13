
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
      std::vector<std::string> instanceUuids;
      std::vector<std::vector<double>> boundingBoxes;

      labels.reserve(categoryLabels.labels_.size());
      instanceUuids.reserve(categoryLabels.labels_.size());
      boundingBoxes.reserve(categoryLabels.labels_.size());

      for (auto& instanceLabel : categoryLabels.labels_)
      {
        labels.push_back(instanceLabel.label_.label_);
        instanceUuids.push_back(instanceLabel.label_.instanceUuid_);

        std::vector<double> box(2 * NumDimensions);

        for (int i = 0; i < NumDimensions; i++)
        {
          box[i] = instanceLabel.minPoint_[i];
          box[i + NumDimensions] = instanceLabel.maxPoint_[i];
        }

        boundingBoxes.push_back(box);
      }

      HighFive::DataSpace labelsDataSpace = HighFive::DataSpace::From(labels);
      auto datasetLabels = getHdf5DataSet<std::string>(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB +
                                                           "_" + categoryLabels.category_,
                                                       labelsDataSpace);
      datasetLabels->write(labels);

      HighFive::DataSpace instancesDataSpace = HighFive::DataSpace::From(instanceUuids);
      auto datasetInstances = getHdf5DataSet<std::string>(
          dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + categoryLabels.category_,
          instancesDataSpace);
      datasetInstances->write(instanceUuids);

      HighFive::DataSpace boxesDataSpace = HighFive::DataSpace::From(boundingBoxes);
      auto datasetBoxes = getHdf5DataSet<double>(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES +
                                                     "_" + categoryLabels.category_,
                                                 boxesDataSpace);
      datasetBoxes->write(boundingBoxes);
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
      auto datasetInstances =
          getHdf5DataSet(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES + "_" + category);
      auto datasetBoxes =
          getHdf5DataSet(dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES + "_" + category);

      if (datasetLabels == nullptr || datasetInstances == nullptr || datasetBoxes == nullptr)
      {
        throw std::invalid_argument("cannot find all bounding box label datasets for category " + category);
      }

      std::vector<std::string> labels;
      std::vector<std::string> instanceUuids;
      std::vector<std::vector<double>> boundingBoxes;

      datasetLabels->read(labels);
      datasetInstances->read(instanceUuids);
      datasetBoxes->read(boundingBoxes);

      if (labels.size() != instanceUuids.size() || labels.size() != boundingBoxes.size())
      {
        throw std::invalid_argument("amounts for labels, instance uuids and bounding boxes do not match for category " +
                                    category);
      }

      for (std::size_t i = 0; i < labels.size(); i++)
      {
        if (boundingBoxes[i].size() != 2 * NumDimensions)
        {
          throw std::invalid_argument(std::to_string(NumDimensions) +
                                      "-dimensional bounding box expected for category " + category + " but got " +
                                      std::to_string(boundingBoxes[i].size() / 2) + "-dimensional one");
        }
        std::array<double, NumDimensions> minPoint;
        std::array<double, NumDimensions> maxPoint;

        for (int j = 0; j < NumDimensions; j++)
        {
          minPoint[j] = boundingBoxes[i][j];
          maxPoint[j] = boundingBoxes[i][j + NumDimensions];
        }

        InstanceLabel label(labels[i], instanceUuids[i]);
        BoundingBoxLabel<NumDimensions> bbLabel(label, minPoint, maxPoint);
        bbLabels[bbLabels.size() - 1].addLabel(bbLabel);
      }
    }
  }

  return bbLabels;
}

} /* namespace seerep_hdf5_py */
