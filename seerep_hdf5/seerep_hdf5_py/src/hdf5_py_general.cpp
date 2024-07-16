#include "seerep_hdf5_py/hdf5_py_general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{
void Hdf5FileWrapper::createProject(const std::string& projectName,
                                    const std::string& rootFrameId)
{
  Hdf5PyGeneral io(*this);

  io.writeProjectname(projectName);
  io.writeProjectFrameId(rootFrameId);
}

void Hdf5FileWrapper::setProjectGeolocation(const std::string& coordinateSystem,
                                            const std::string& ellipsoid,
                                            double latitude, double longitude,
                                            double altitude)
{
  Hdf5PyGeneral io(*this);

  io.setProjectGeolocation(coordinateSystem, ellipsoid, latitude, longitude,
                           altitude);
}

Hdf5PyGeneral::Hdf5PyGeneral(Hdf5FileWrapper& hdf5File)
  : Hdf5CoreGeneral(hdf5File.getFile(), hdf5File.getMutex())
{
}

void Hdf5PyGeneral::setProjectGeolocation(const std::string& coordinateSystem,
                                          const std::string& ellipsoid,
                                          double latitude, double longitude,
                                          double altitude)
{
  const std::scoped_lock lock(*m_write_mtx);

  writeAttributeToHdf5(*m_file, GEODETICLOCATION_COORDINATESYSTEM,
                       coordinateSystem);
  writeAttributeToHdf5(*m_file, GEODETICLOCATION_ELLIPSOID, ellipsoid);
  writeAttributeToHdf5(*m_file, GEODETICLOCATION_LATITUDE, latitude);
  writeAttributeToHdf5(*m_file, GEODETICLOCATION_LONGITUDE, longitude);
  writeAttributeToHdf5(*m_file, GEODETICLOCATION_ALTITUDE, altitude);

  m_file->flush();
}

void Hdf5PyGeneral::writeLabelsGeneral(
    const std::string& dataGroupId,
    const std::vector<seerep_hdf5_py::GeneralLabel>& generalLabels)
{
  for (const auto& categoryLabels : generalLabels)
  {
    if (!categoryLabels.labels_.empty())
    {
      std::vector<std::string> labels;
      std::vector<float> labelConfidences;
      std::vector<std::string> instanceUuids;

      labels.reserve(categoryLabels.labels_.size());
      instanceUuids.reserve(categoryLabels.labels_.size());

      for (const auto& instanceLabel : categoryLabels.labels_)
      {
        labels.push_back(instanceLabel.label_);
        labelConfidences.push_back(instanceLabel.confidence_);
        instanceUuids.push_back(instanceLabel.instanceUuid_);
      }

      HighFive::DataSpace labelsDataSpace = HighFive::DataSpace::From(labels);
      auto datasetLabels = getHdf5DataSet<std::string>(
          dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL +
              "_" + categoryLabels.category_,
          labelsDataSpace);
      datasetLabels->write(labels);

      HighFive::DataSpace labelConfidencesDataSpace =
          HighFive::DataSpace::From(labelConfidences);
      auto datasetLabelConfidences = getHdf5DataSet<float>(
          dataGroupId + "/" +
              seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALCONFIDENCES + "_" +
              categoryLabels.category_,
          labelConfidencesDataSpace);
      datasetLabelConfidences->write(labelConfidences);

      HighFive::DataSpace instancesDataSpace =
          HighFive::DataSpace::From(instanceUuids);
      auto datasetInstances = getHdf5DataSet<std::string>(
          dataGroupId + "/" +
              seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES + "_" +
              categoryLabels.category_,
          instancesDataSpace);
      datasetInstances->write(instanceUuids);
    }
  }
  m_file->flush();
}

std::vector<GeneralLabel>
Hdf5PyGeneral::readLabelsGeneral(const std::string& dataGroupId)
{
  auto hdf5GroupPtr = getHdf5Group(dataGroupId);

  std::vector<GeneralLabel> generalLabels;

  const std::vector<std::string> groupDatasets =
      hdf5GroupPtr->listObjectNames();
  for (const auto& datasetName : groupDatasets)
  {
    if (datasetName.rfind(
            seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES, 0) == 0)
    {
      std::string category = datasetName.substr(
          seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES.length() + 1);
      generalLabels.push_back(GeneralLabel(category));

      auto datasetLabels = getHdf5DataSet(
          dataGroupId + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERAL +
          "_" + category);
      auto datasetLabelConfidences = getHdf5DataSet(
          dataGroupId + "/" +
          seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALCONFIDENCES + "_" +
          category);
      auto datasetInstances = getHdf5DataSet(
          dataGroupId + "/" +
          seerep_hdf5_core::Hdf5CoreGeneral::LABELGENERALINSTANCES + "_" +
          category);

      if (datasetLabels == nullptr || datasetInstances == nullptr)
      {
        throw std::invalid_argument(
            "cannot find all general label datasets for category " + category);
      }

      std::vector<std::string> labels;
      std::vector<float> labelConfidences;
      std::vector<std::string> instanceUuids;

      datasetLabels->read(labels);
      datasetLabelConfidences->read(labelConfidences);
      datasetInstances->read(instanceUuids);

      if (labels.size() != instanceUuids.size() ||
          labels.size() != labelConfidences.size())
      {
        throw std::invalid_argument(
            "amounts for labels and instance uuids do not match for category " +
            category);
      }

      for (std::size_t i = 0; i < labels.size(); i++)
      {
        InstanceLabel label(labels[i], labelConfidences[i], instanceUuids[i]);
        generalLabels[generalLabels.size() - 1].addLabel(label);
      }
    }
  }

  return generalLabels;
}

} /* namespace seerep_hdf5_py */
