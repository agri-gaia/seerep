#include "seerep_hdf5_fb/hdf5_fb_general.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_fb
{
Hdf5FbGeneral::Hdf5FbGeneral(std::shared_ptr<HighFive::File>& file,
                             std::shared_ptr<std::mutex>& write_mtx)
  : seerep_hdf5_core::Hdf5CoreGeneral(file, write_mtx)
{
}

void Hdf5FbGeneral::writeAttributeMap(
    const std::shared_ptr<HighFive::DataSet> dataSetPtr,
    const flatbuffers::Vector<flatbuffers::Offset<seerep::fb::UnionMapEntry>>*
        attributes)
{
  if (attributes)
  {
    for (auto attribute : *attributes)
    {
      if (attribute->value_type() == seerep::fb::Datatypes_Boolean)
      {
        writeAttributeToHdf5(
            *dataSetPtr, attribute->key()->str(),
            static_cast<const seerep::fb::Boolean*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_Integer)
      {
        writeAttributeToHdf5(
            *dataSetPtr, attribute->key()->str(),
            static_cast<const seerep::fb::Integer*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_Double)
      {
        writeAttributeToHdf5(
            *dataSetPtr, attribute->key()->str(),
            static_cast<const seerep::fb::Double*>(attribute->value())->data());
      }
      else if (attribute->value_type() == seerep::fb::Datatypes_String)
      {
        writeAttributeToHdf5(
            *dataSetPtr, attribute->key()->str(),
            static_cast<const seerep::fb::String*>(attribute->value())
                ->data()
                ->str());
      }
      else
      {
        std::stringstream errorMsg;
        errorMsg << "type " << attribute->value_type() << " of attribute "
                 << attribute->key()->c_str() << " not implemented in hdf5-io.";
        BOOST_LOG_SEV(m_logger, boost::log::trivial::severity_level::error)
            << errorMsg.str();
        throw std::invalid_argument(errorMsg.str());
      }
    }
  }
}

void Hdf5FbGeneral::writeLabelsFb(const std::string& datatypeGroup,
                                  const std::string& uuid,
                                  const LabelsCategoryFb* labelCategoryVectorFb)
{
  if (labelCategoryVectorFb && labelCategoryVectorFb->size() != 0)
  {
    std::vector<seerep_core_msgs::LabelCategory> labelCategoryVector;
    for (auto labelsCategory : *labelCategoryVectorFb)
    {
      std::vector<std::string> labels, instances;
      std::vector<int> labelsIdDatumaro, instancesIdDatumaro;
      for (auto label : *labelsCategory->labels())
      {
        labels.push_back(label->label()->str());
        labelsIdDatumaro.push_back(label->labelIdDatumaro());
        if (label->instanceUuid())
        {
          instances.push_back(label->instanceUuid()->str());
        }
        if (label->instanceIdDatumaro())
        {
          instancesIdDatumaro.push_back(label->instanceIdDatumaro());
        }
      }
      seerep_core_msgs::LabelCategory labelCategory;
      labelCategory.category = labelsCategory->category()->c_str();
      labelCategory.labels = labels;
      labelCategory.labelsIdDatumaro = labelsIdDatumaro;
      labelCategory.instances = instances;
      labelCategory.instancesIdDatumaro = instancesIdDatumaro;
      labelCategory.datumaroJson = labelsCategory->datumaroJson()->c_str();
      labelCategoryVector.push_back(labelCategory);
    }
    Hdf5CoreGeneral::writeLabels(datatypeGroup, uuid, labelCategoryVector);
  }
}

std::optional<flatbuffers::Offset<LabelsCategoryFb>>
Hdf5FbGeneral::readLabels(const std::string& datatypeGroup,
                          const std::string& uuid,
                          flatbuffers::grpc::MessageBuilder& builder)
{
  std::vector<std::string> labelCategories, datumaroJsonPerCategory;
  std::vector<std::vector<seerep_core_msgs::Label>> labelsCategory;

  seerep_hdf5_core::Hdf5CoreGeneral::readLabels(datatypeGroup, uuid,
                                                labelCategories, labelsCategory,
                                                datumaroJsonPerCategory);

  if (labelsCategory.empty())
  {
    return std::nullopt;
  }

  std::vector<flatbuffers::Offset<seerep::fb::LabelCategory>> labelCategory;
  labelCategory.reserve(labelCategories.size());

  // loop categories
  for (size_t icat = 0; icat < labelCategories.size(); icat++)
  {
    std::vector<flatbuffers::Offset<seerep::fb::Label>> labelVector;
    labelVector.reserve(labelsCategory.at(icat).size());

    // loop labels
    for (size_t i = 0; i < labelsCategory.at(icat).size(); i++)
    {
      auto labelStr = builder.CreateString(labelsCategory.at(icat).at(i).label);
      auto instanceOffset =
          builder.CreateString(boost::lexical_cast<std::string>(
              labelsCategory.at(icat).at(i).uuidInstance));

      seerep::fb::LabelBuilder labelBuilder(builder);
      labelBuilder.add_label(labelStr);
      labelBuilder.add_labelIdDatumaro(
          labelsCategory.at(icat).at(i).labelIdDatumaro);
      labelBuilder.add_instanceUuid(instanceOffset);
      labelBuilder.add_instanceIdDatumaro(
          labelsCategory.at(icat).at(i).instanceIdDatumaro);
      labelVector.push_back(labelBuilder.Finish());
    }

    auto labelOffset =
        builder.CreateVector<flatbuffers::Offset<seerep::fb::Label>>(
            labelVector);

    auto categoryOffset = builder.CreateString(labelCategories.at(icat));

    auto datumaroJsonOffset =
        builder.CreateString(datumaroJsonPerCategory.at(icat));

    seerep::fb::LabelCategoryBuilder labelsCategoryBuilder(builder);
    labelsCategoryBuilder.add_category(categoryOffset);
    labelsCategoryBuilder.add_labels(labelOffset);
    labelsCategoryBuilder.add_datumaroJson(datumaroJsonOffset);

    auto labelsOfCategory = labelsCategoryBuilder.Finish();
    labelCategory.push_back(labelsOfCategory);
  }

  return std::optional<flatbuffers::Offset<LabelsCategoryFb>>(
      builder.CreateVector<flatbuffers::Offset<seerep::fb::LabelCategory>>(
          labelCategory));
}
}  // namespace seerep_hdf5_fb
