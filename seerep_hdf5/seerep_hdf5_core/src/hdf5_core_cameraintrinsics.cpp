#include "seerep_hdf5_core/hdf5_core_cameraintrinsics.h"

namespace seerep_hdf5_core
{
Hdf5CoreCameraIntrinsics::Hdf5CoreCameraIntrinsics(
    std::shared_ptr<HighFive::File>& file,
    std::shared_ptr<std::mutex>& write_mtx)
  : Hdf5CoreGeneral(file, write_mtx)
{
}

void Hdf5CoreCameraIntrinsics::writeCameraIntrinsics(
    const seerep_core_msgs::camera_intrinsics& camIntrinsics)
{
  // the storage path has to be determined using the uuid of the cam intriniscs,
  // which is available inside the header
  std::string id =
      boost::lexical_cast<std::string>(camIntrinsics.header.uuidData);
  std::string hdf5GroupPath = cameraIntrinsicPath(id);

  std::shared_ptr<HighFive::Group> dataGroupPtr =
      getHdf5Group(hdf5GroupPath, true);

  if (dataGroupPtr)
  {
    writeHeader<HighFive::Group>(*dataGroupPtr, camIntrinsics.header);
    writeAttributeToHdf5<uint32_t>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::HEIGHT,
        camIntrinsics.height);
    writeAttributeToHdf5<uint32_t>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::WIDTH,
        camIntrinsics.width);
    writeAttributeToHdf5<std::string>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::DISTORTION_MODEL,
        camIntrinsics.distortion_model);
    writeAttributeToHdf5<std::vector<double>>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::DISTORTION,
        camIntrinsics.distortion);
    writeAttributeToHdf5<std::vector<double>>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::INTRINSIC_MATRIX,
        camIntrinsics.intrinsic_matrix);
    writeAttributeToHdf5<std::vector<double>>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::RECTIFICATION_MATRIX,
        camIntrinsics.rectification_matrix);
    writeAttributeToHdf5<std::vector<double>>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::PROJECTION_MATRIX,
        camIntrinsics.projection_matrix);
    writeAttributeToHdf5<uint32_t>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::BINNING_X,
        camIntrinsics.binning_x);
    writeAttributeToHdf5<uint32_t>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::BINNING_Y,
        camIntrinsics.binning_y);

    // region of interest
    writeAttributeToHdf5<uint32_t>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_X_OFFSET,
        camIntrinsics.region_of_interest.x_offset);
    writeAttributeToHdf5<uint32_t>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_Y_OFFSET,
        camIntrinsics.region_of_interest.y_offset);
    writeAttributeToHdf5<uint32_t>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_HEIGHT,
        camIntrinsics.region_of_interest.height);
    writeAttributeToHdf5<uint32_t>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_WIDTH,
        camIntrinsics.region_of_interest.width);
    writeAttributeToHdf5<bool>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_DO_RECTIFY,
        camIntrinsics.region_of_interest.do_rectify);

    writeAttributeToHdf5<double>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::MAX_VIEWING_DISTANCE,
        camIntrinsics.maximum_viewing_distance);
  }

  m_file->flush();
}

seerep_core_msgs::camera_intrinsics
Hdf5CoreCameraIntrinsics::readCameraIntrinsics(
    const boost::uuids::uuid& cameraintrinsics_uuid)
{
  const std::scoped_lock lock(*m_write_mtx);

  std::string id = boost::lexical_cast<std::string>(cameraintrinsics_uuid);

  std::string hdf5GroupPath = cameraIntrinsicPath(id);

  auto dataGroupPtr = getHdf5Group(hdf5GroupPath, false);

  seerep_core_msgs::camera_intrinsics ci;

  if (dataGroupPtr)
  {
    readHeader(id, *dataGroupPtr, ci.header);

    ci.height = readAttributeFromHdf5<uint32_t>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::HEIGHT, id);
    ci.width = readAttributeFromHdf5<uint32_t>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::WIDTH, id);
    ci.distortion_model = readAttributeFromHdf5<std::string>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::DISTORTION_MODEL, id);

    // read distortion
    dataGroupPtr
        ->getAttribute(seerep_hdf5_core::Hdf5CoreCameraIntrinsics::DISTORTION)
        .read<std::vector<double>>(ci.distortion);

    // read intrinsic matrix
    dataGroupPtr
        ->getAttribute(
            seerep_hdf5_core::Hdf5CoreCameraIntrinsics::INTRINSIC_MATRIX)
        .read<std::vector<double>>(ci.intrinsic_matrix);

    // read rectification matrix
    dataGroupPtr
        ->getAttribute(
            seerep_hdf5_core::Hdf5CoreCameraIntrinsics::RECTIFICATION_MATRIX)
        .read<std::vector<double>>(ci.rectification_matrix);

    // read projection matrix
    dataGroupPtr
        ->getAttribute(
            seerep_hdf5_core::Hdf5CoreCameraIntrinsics::PROJECTION_MATRIX)
        .read<std::vector<double>>(ci.projection_matrix);

    ci.binning_x = readAttributeFromHdf5<uint32_t>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::BINNING_X,
        id);
    ci.binning_y = readAttributeFromHdf5<uint32_t>(
        *dataGroupPtr, seerep_hdf5_core::Hdf5CoreCameraIntrinsics::BINNING_Y,
        id);

    ci.region_of_interest.x_offset = readAttributeFromHdf5<uint32_t>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_X_OFFSET,
        id);
    ci.region_of_interest.y_offset = readAttributeFromHdf5<uint32_t>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_Y_OFFSET,
        id);
    ci.region_of_interest.height = readAttributeFromHdf5<uint32_t>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_HEIGHT,
        id);
    ci.region_of_interest.width = readAttributeFromHdf5<uint32_t>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_WIDTH,
        id);
    ci.region_of_interest.do_rectify = readAttributeFromHdf5<bool>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::REGION_OF_INTEREST_DO_RECTIFY,
        id);

    ci.maximum_viewing_distance = readAttributeFromHdf5<double>(
        *dataGroupPtr,
        seerep_hdf5_core::Hdf5CoreCameraIntrinsics::MAX_VIEWING_DISTANCE, id);
  }

  return ci;
}

bool Hdf5CoreCameraIntrinsics::cameraIntrinsicExists(
    const boost::uuids::uuid& camera_intrinsic_uuid) const
{
  return exists(cameraIntrinsicPath(
      boost::lexical_cast<std::string>(camera_intrinsic_uuid)));
}

const std::string Hdf5CoreCameraIntrinsics::cameraIntrinsicPath(
    const std::string& camera_instrinsic_uuid) const
{
  return HDF5_GROUP_CAMINTRINSICS + "/" + camera_instrinsic_uuid;
}

}  // namespace seerep_hdf5_core
