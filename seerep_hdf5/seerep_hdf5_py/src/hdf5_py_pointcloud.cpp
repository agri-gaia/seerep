#include "seerep_hdf5_py/hdf5_py_pointcloud.h"

#include <highfive/H5DataSet.hpp>

namespace seerep_hdf5_py
{

Hdf5PyPointCloud::Hdf5PyPointCloud(Hdf5FileWrapper& hdf5File)
  : Hdf5CoreGeneral(hdf5File.getFile(), hdf5File.getMutex())
  , Hdf5PyGeneral(hdf5File)
{
}

std::vector<std::string> Hdf5PyPointCloud::getPointClouds()
{
  const std::scoped_lock lock(*m_write_mtx);

  if (!m_file->exist(
          seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD))
  {
    return std::vector<std::string>();
  }
  const HighFive::Group& cloudsGroup = m_file->getGroup(
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD);
  return cloudsGroup.listObjectNames();
}

void Hdf5PyPointCloud::writePointCloud(
    const std::string& uuid, const std::string& frameId, int64_t seconds,
    int32_t nanos, uint32_t sequence,
    const std::map<std::string, py::array> fields,
    const std::vector<GeneralLabel>& generalLabels,
    const std::vector<CategorizedBoundingBoxLabel<3>>& bbLabels)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string cloudGroupId =
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;

  std::shared_ptr<HighFive::Group> dataGroupPtr = getHdf5Group(cloudGroupId);

  std::size_t numPoints = 0;
  for (const auto& [fieldName, fieldData] : fields)
  {
    py::buffer_info fieldBuffInfo = fieldData.request();
    if (fieldName.compare("xyz") == 0)
    {
      numPoints = fieldBuffInfo.shape[0];
      break;
    }
    else if (fieldName.compare("x") == 0)
    {
      numPoints = fieldBuffInfo.shape[0];
      break;
    }
    else if (fieldName.compare("points") == 0)
    {
      numPoints = fieldBuffInfo.shape[0];
      break;
    }
  }

  if (numPoints == 0)
  {
    throw std::invalid_argument("no points found in fields. does a field "
                                "'xyz', 'x' or 'points' exist?");
  }

  // write header

  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::HEADER_FRAME_ID,
                       frameId);
  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::HEADER_SEQ,
                       sequence);
  writeAttributeToHdf5(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::HEADER_STAMP_SECONDS,
      seconds);
  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::HEADER_STAMP_NANOS,
                       nanos);
  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::HEADER_SEQ,
                       sequence);

  // write labels

  Hdf5PyGeneral::writeBoundingBoxLabeled(cloudGroupId, bbLabels);
  Hdf5PyGeneral::writeLabelsGeneral(cloudGroupId, generalLabels);

  // write data

  for (const auto& [fieldName, fieldData] : fields)
  {
    writeField(cloudGroupId, fieldName, fields);
  }

  // write boundingbox

  bool success = false;

  if (!success)
  {
    success = writeBoundingBox<double, 3>(cloudGroupId, { "points" }, fields);
  }
  if (!success)
  {
    success = writeBoundingBox<float, 3>(cloudGroupId, { "points" }, fields);
  }
  if (!success)
  {
    success = writeBoundingBox<double, 3>(cloudGroupId, { "xyz" }, fields);
  }
  if (!success)
  {
    success = writeBoundingBox<float, 3>(cloudGroupId, { "xyz" }, fields);
  }
  if (!success)
  {
    success =
        writeBoundingBox<double, 3>(cloudGroupId, { "x", "y", "z" }, fields);
  }
  if (!success)
  {
    success =
        writeBoundingBox<float, 3>(cloudGroupId, { "x", "y", "z" }, fields);
  }

  // write point fields

  std::vector<std::string> names(fields.size());
  std::vector<uint32_t> offsets(fields.size());
  std::vector<uint8_t> datatypes(fields.size());
  std::vector<uint32_t> counts(fields.size());
  uint32_t offset = 0;
  for (const auto& [fieldName, fieldData] : fields)
  {
    py::buffer_info fieldBuffInfo = fieldData.request();

    names.push_back(fieldName);
    offsets.push_back(offset);

    if (fieldData.dtype().equal(py::dtype::of<int8_t>()))
    {
      datatypes.push_back(1);  // int8
    }
    else if (fieldData.dtype().equal(py::dtype::of<uint8_t>()))
    {
      datatypes.push_back(1);  // uint8
    }
    else if (fieldData.dtype().equal(py::dtype::of<uint32_t>()))
    {
      datatypes.push_back(1);  // uint32
    }
    else if (fieldData.dtype().equal(py::dtype::of<float>()))
    {
      datatypes.push_back(1);  // float32
    }
    else if (fieldData.dtype().equal(py::dtype::of<double>()))
    {
      datatypes.push_back(1);  // float64
    }

    uint32_t numElements = 1;
    if (fieldBuffInfo.shape.size() > 1)
    {
      numElements = fieldBuffInfo.shape[1];
    }
    counts.push_back(numElements);

    offset += fieldData.dtype().itemsize() * numElements;
  }

  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME, names);
  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET,
                       offsets);
  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE,
                       datatypes);
  writeAttributeToHdf5(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT, counts);

  // write basic attributes

  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::HEIGHT, 1);
  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::WIDTH, numPoints);
  writeAttributeToHdf5(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::IS_BIGENDIAN, false);
  writeAttributeToHdf5(
      *dataGroupPtr, seerep_hdf5_core::Hdf5CorePointCloud::POINT_STEP, offset);
  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::ROW_STEP,
                       numPoints * offset);
  writeAttributeToHdf5(*dataGroupPtr,
                       seerep_hdf5_core::Hdf5CorePointCloud::IS_DENSE, true);

  m_file->flush();
}

std::tuple<std::map<std::string, py::array>, std::vector<GeneralLabel>,
           std::vector<CategorizedBoundingBoxLabel<3>>>
Hdf5PyPointCloud::readPointCloud(const std::string& uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string cloudGroupId =
      seerep_hdf5_core::Hdf5CorePointCloud::HDF5_GROUP_POINTCLOUD + "/" + uuid;

  if (!m_file->exist(cloudGroupId))
  {
    return std::make_tuple(std::map<std::string, py::array>(),
                           std::vector<GeneralLabel>(),
                           std::vector<CategorizedBoundingBoxLabel<3>>());
  }

  HighFive::Group cloudGroup = m_file->getGroup(cloudGroupId);

  std::vector<std::string> names;
  std::vector<uint32_t> offsets;
  std::vector<uint32_t> counts;
  std::vector<uint8_t> datatypes;

  cloudGroup.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_NAME)
      .read(names);
  cloudGroup.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_OFFSET)
      .read(offsets);
  cloudGroup.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_DATATYPE)
      .read(datatypes);
  cloudGroup.getAttribute(seerep_hdf5_core::Hdf5CorePointCloud::FIELD_COUNT)
      .read(counts);

  if (names.size() != offsets.size() || names.size() != counts.size() ||
      names.size() != datatypes.size())
  {
    throw std::invalid_argument(
        "point field sizes do not match up for pointcloud " + uuid);
  }

  std::map<std::string, py::array> fields;

  for (std::size_t i = 0; i < names.size(); i++)
  {
    if (counts[i] == 0)
    {
      continue;
    }

    if (!m_file->exist(cloudGroupId + "/" + names[i]))
    {
      throw std::invalid_argument("pointcloud field " + names[i] +
                                  " not found in pointcloud " + uuid);
    }

    std::shared_ptr<HighFive::DataSet> dataset =
        getHdf5DataSet(cloudGroupId + "/" + names[i]);

    if (datatypes[i] == 1)
    {
      fields[names[i]] = readField<int8_t>(dataset);
    }
    else if (datatypes[i] == 5)
    {
      fields[names[i]] = readField<int32_t>(dataset);
    }
    else if (datatypes[i] == 2)
    {
      fields[names[i]] = readField<uint8_t>(dataset);
    }
    else if (datatypes[i] == 6)
    {
      fields[names[i]] = readField<uint32_t>(dataset);
    }
    else if (datatypes[i] == 7)
    {
      fields[names[i]] = readField<float>(dataset);
    }
    else if (datatypes[i] == 7)
    {
      fields[names[i]] = readField<double>(dataset);
    }
    else
    {
      throw std::invalid_argument("unknown type in field " + names[i] +
                                  " of pointcloud " + uuid);
    }
  }

  // read labels
  auto generalLabels = Hdf5PyGeneral::readLabelsGeneral(cloudGroupId);
  auto bbLabels = Hdf5PyGeneral::readBoundingBoxLabeled<3>(cloudGroupId);

  return std::make_tuple(fields, generalLabels, bbLabels);
}

void Hdf5PyPointCloud::writeField(const std::string& cloudGroupId,
                                  const std::string& fieldName,
                                  const std::map<std::string, py::array>& fields)
{
  if (!writeFieldTyped<char, int, uint8_t, unsigned int, float, double>(
          cloudGroupId, fieldName, fieldName, fields))
  {
    throw std::invalid_argument("unable to write field '" + fieldName + "'");
  }
}

} /* namespace seerep_hdf5_py */
