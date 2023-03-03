
#include <gtest/gtest.h>
#include <seerep-hdf5-core/hdf5-core-cameraintrinsics.h>
#include <seerep-msgs/camera_intrinsics.h>
#include <seerep-msgs/datatype.h>
#include <seerep-msgs/region_of_interest.h>
#include <seerep-msgs/timestamp.h>

#include <H5File.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <filesystem>

namespace seerep_hdf5_core
{
namespace tests
{

const seerep_core_msgs::region_of_interest createRegionOfInterest()
{
  seerep_core_msgs::region_of_interest roi;

  roi.do_rectify = false;
  roi.height = 54;
  roi.width = 23;
  roi.x_offset = 88;
  roi.y_offset = 32;

  return roi;
}

const seerep_core_msgs::Timestamp createTimestamp()
{
  seerep_core_msgs::Timestamp ts;

  ts.nanos = 3456;
  ts.seconds = 678;

  return ts;
}

const seerep_core_msgs::Header createHeader(const boost::uuids::uuid& projectUUID, const boost::uuids::uuid& messageUUID)
{
  seerep_core_msgs::Header header;

  header.datatype = seerep_core_msgs::Datatype::Point;
  header.frameId = "frameid";
  header.sequence = 34;
  header.timestamp = createTimestamp();
  header.uuidData = messageUUID;
  header.uuidProject = projectUUID;

  return header;
}

seerep_core_msgs::camera_intrinsics createCameraIntinsicsMessage(const boost::uuids::uuid& projectUUID,
                                                                 const boost::uuids::uuid& messageUUID)
{
  seerep_core_msgs::camera_intrinsics ci;

  ci.header = createHeader(projectUUID, messageUUID);

  ci.height = 4;
  ci.width = 5;

  ci.distortion_model = "plumb_bob";
  ci.distortion = { 4, 5, 6, 7, 8 };

  std::vector<double> intrinsics_matrix, rectification_matrix, projection_matrix;

  for (size_t i = 0; i < 10; i++)
  {
    ci.intrinsic_matrix.push_back(0.01 + i);
    ci.rectification_matrix.push_back(0.02 + i);
    ci.projection_matrix.push_back(0.03 + i);
  }

  ci.binning_x = 6;
  ci.binning_y = 8;

  ci.region_of_interest = createRegionOfInterest();

  return ci;
}

class coreWriteLoadTest : public testing::Test
{
protected:
  static std::shared_ptr<std::mutex> hdf5FileMutex;
  static std::string hdf5FileName;
  static std::shared_ptr<HighFive::File> hdf5File;
  static std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> ciIO;

  static boost::uuids::uuid projectUUID;
  static boost::uuids::uuid messageUUID;
  static std::string projectName;

  static seerep_core_msgs::camera_intrinsics writeCI;
  static seerep_core_msgs::camera_intrinsics readCI;

  static void SetUpTestSuite()
  {
    projectName = "testProject";
    projectUUID = boost::uuids::random_generator()();
    messageUUID = boost::uuids::random_generator()();

    hdf5FileMutex = std::make_shared<std::mutex>();
    hdf5FileName = boost::lexical_cast<std::string>(projectUUID) + ".h5";
    hdf5File = std::make_shared<HighFive::File>(hdf5FileName, HighFive::File::ReadWrite | HighFive::File::Create);

    ciIO = std::make_shared<seerep_hdf5_core::Hdf5CoreCameraIntrinsics>(hdf5File, hdf5FileMutex);

    writeCI = createCameraIntinsicsMessage(projectUUID, messageUUID);
    ciIO->writeCameraIntrinsics(writeCI);

    readCI = ciIO->readCameraIntrinsics(projectUUID, messageUUID);
  }

  static void TearDownTestSuite()
  {
    std::filesystem::remove(hdf5FileName);
  }
};

std::shared_ptr<std::mutex> coreWriteLoadTest::hdf5FileMutex;
std::string coreWriteLoadTest::hdf5FileName;
std::shared_ptr<HighFive::File> coreWriteLoadTest::hdf5File;
std::shared_ptr<seerep_hdf5_core::Hdf5CoreCameraIntrinsics> coreWriteLoadTest::ciIO;

boost::uuids::uuid coreWriteLoadTest::projectUUID;
boost::uuids::uuid coreWriteLoadTest::messageUUID;
std::string coreWriteLoadTest::projectName;

seerep_core_msgs::camera_intrinsics coreWriteLoadTest::writeCI;
seerep_core_msgs::camera_intrinsics coreWriteLoadTest::readCI;

void testHeader(const seerep_core_msgs::Header readHeader, const seerep_core_msgs::Header writeHeader)
{
  EXPECT_EQ(readHeader.timestamp.seconds, writeHeader.timestamp.seconds);
  EXPECT_EQ(readHeader.timestamp.nanos, writeHeader.timestamp.nanos);
  EXPECT_EQ(readHeader.frameId, writeHeader.frameId);
  EXPECT_EQ(readHeader.uuidProject, writeHeader.uuidProject);
  EXPECT_EQ(readHeader.uuidData, writeHeader.uuidData);
}

void testRegionOfInterest(const seerep_core_msgs::region_of_interest readROI,
                          const seerep_core_msgs::region_of_interest writeROI)
{
  EXPECT_EQ(readROI.x_offset, writeROI.x_offset);
  EXPECT_EQ(readROI.y_offset, writeROI.y_offset);
  EXPECT_EQ(readROI.height, writeROI.height);
  EXPECT_EQ(readROI.width, writeROI.width);
  EXPECT_EQ(readROI.do_rectify, writeROI.do_rectify);
}

TEST_F(coreWriteLoadTest, testCameraIntrinsics)
{
  testHeader(readCI.header, writeCI.header);

  EXPECT_EQ(readCI.height, writeCI.height);
  EXPECT_EQ(readCI.width, writeCI.width);
  EXPECT_EQ(readCI.distortion_model, writeCI.distortion_model);

  for (size_t i = 0; i < writeCI.distortion.size(); i++)
  {
    EXPECT_EQ(readCI.distortion[i], writeCI.distortion[i]);
  }

  for (size_t i = 0; i < writeCI.intrinsic_matrix.size(); i++)
  {
    EXPECT_EQ(readCI.intrinsic_matrix[i], writeCI.intrinsic_matrix[i]);
  }

  for (size_t i = 0; i < writeCI.rectification_matrix.size(); i++)
  {
    EXPECT_EQ(readCI.rectification_matrix[i], writeCI.rectification_matrix[i]);
  }

  for (size_t i = 0; i < writeCI.projection_matrix.size(); i++)
  {
    EXPECT_EQ(readCI.projection_matrix[i], writeCI.projection_matrix[i]);
  }

  EXPECT_EQ(readCI.binning_x, writeCI.binning_x);
  EXPECT_EQ(readCI.binning_y, writeCI.binning_y);

  testRegionOfInterest(readCI.region_of_interest, writeCI.region_of_interest);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace tests
}  // namespace seerep_hdf5_core
