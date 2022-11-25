#include "seerep-hdf5-fb/hdf5-fb-general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbGeneral::Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file, std::shared_ptr<std::mutex>& write_mtx)
  : seerep_hdf5_core::Hdf5CoreGeneral(file, write_mtx)
{
}

void Hdf5FbGeneral::writeAttributeMap(
    const std::shared_ptr<HighFive::DataSet> dataSetPtr,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>>* attributes)
{
  if (attributes)
  {
    for (auto attribute : *attributes)
    {
      if (attribute->value_type() == seerep::fb::Datatypes_Boolean)
      {
        writeAttributeToHdf5(*dataSetPtr, attribute->key()->str(),
                             static_cast<const seerep::fb::Boolean*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_Integer)
      {
        writeAttributeToHdf5(*dataSetPtr, attribute->key()->str(),
                             static_cast<const seerep::fb::Integer*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_Double)
      {
        writeAttributeToHdf5(*dataSetPtr, attribute->key()->str(),
                             static_cast<const seerep::fb::Double*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_String)
      {
        writeAttributeToHdf5(*dataSetPtr, attribute->key()->str(),
                             static_cast<const seerep::fb::String*>(attribute->value())->data()->str());
      }
      else
      {
        std::stringstream errorMsg;
        errorMsg << "type " << attribute->value_type() << " of attribute " << attribute->key()->c_str()
                 << " not implemented in hdf5-io.";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error) << errorMsg.str();
        throw std::invalid_argument(errorMsg.str());
      }
    }
  }
}

void Hdf5FbGeneral::writeBoundingBoxLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBoxLabeled>>* boundingboxLabeled)
{
  if (boundingboxLabeled && boundingboxLabeled->size() != 0)
  {
    std::string id = datatypeGroup + "/" + uuid;

    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : *boundingboxLabeled)
    {
      labels.push_back(label->labelWithInstance()->label()->str());
      std::vector<double> box{ label->bounding_box()->point_min()->x(), label->bounding_box()->point_min()->y(),
                               label->bounding_box()->point_min()->z(), label->bounding_box()->point_max()->x(),
                               label->bounding_box()->point_max()->y(), label->bounding_box()->point_max()->z() };
      boundingBoxes.push_back(box);

      instances.push_back(label->labelWithInstance()->instanceUuid()->str());
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

void Hdf5FbGeneral::writeBoundingBox2DLabeled(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>>* boundingbox2DLabeled)
{
  std::string id = datatypeGroup + "/" + uuid;

  if (boundingbox2DLabeled && boundingbox2DLabeled->size() != 0)
  {
    std::vector<std::string> labels;
    std::vector<std::vector<double>> boundingBoxes;
    std::vector<std::string> instances;
    for (auto label : *boundingbox2DLabeled)
    {
      labels.push_back(label->labelWithInstance()->label()->str());
      std::vector<double> box{ label->bounding_box()->point_min()->x(), label->bounding_box()->point_min()->y(),
                               label->bounding_box()->point_max()->x(), label->bounding_box()->point_max()->y() };
      boundingBoxes.push_back(box);

      instances.push_back(label->labelWithInstance()->instanceUuid()->str());
    }

    if (m_file->exist(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB) ||
        m_file->exist(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES) ||
        m_file->exist(id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES))
    {
      throw std::invalid_argument(datatypeGroup + " " + uuid + " already has bounding box based labels");
    }
    else
    {
      HighFive::DataSet datasetLabels = m_file->createDataSet<std::string>(
          id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBB, HighFive::DataSpace::From(labels));
      datasetLabels.write(labels);

      HighFive::DataSet datasetBoxes = m_file->createDataSet<double>(
          id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBBOXES, HighFive::DataSpace::From(boundingBoxes));
      datasetBoxes.write(boundingBoxes);

      HighFive::DataSet datasetInstances = m_file->createDataSet<std::string>(
          id + "/" + seerep_hdf5_core::Hdf5CoreGeneral::LABELBBINSTANCES, HighFive::DataSpace::From(instances));
      datasetInstances.write(instances);
    }

    m_file->flush();
  }
}

void Hdf5FbGeneral::writeLabelsGeneral(
    const std::string& datatypeGroup, const std::string& uuid,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>>* labelsGeneral)
{
  if (labelsGeneral && labelsGeneral->size() != 0)
  {
    std::vector<std::string> labels;
    std::vector<std::string> instances;
    for (auto label : *labelsGeneral)
    {
      labels.push_back(label->label()->str());

      instances.push_back(label->instanceUuid()->str());
    }
    Hdf5CoreGeneral::writeLabelsGeneral(datatypeGroup, uuid, labels, instances);
  }
}

}  // namespace seerep_hdf5_fb
