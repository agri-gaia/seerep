#include <gtest/gtest.h>
#include <seerep-hdf5-fb/hdf5-fb-image.h>

#include <ctime>
#include <filesystem>
#include <optional>
#include <string>

// seerep flatbuffer messages
#include <seerep-core/core.h>
#include <seerep-msgs/image_generated.h>

#include <H5File.hpp>
#include <boost/filesystem.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

bool fBImageEquals(const seerep::fb::Image* lhs, const seerep::fb::Image* rhs)
{
  return lhs->width() == rhs->width();
}

// consider adding SetUp() and TearDown() for exception safety
class FileSetUp : public testing::Test
{
protected:
  const unsigned int height = 256;
  const unsigned int width = 256;
  const std::string imgId = "test1";
  boost::uuids::uuid projectUUID;

  std::shared_ptr<std::mutex> hdf5FileMutex;
  std::shared_ptr<HighFive::File> hdf5File;
  std::string hdf5FileName;

  FileSetUp()
  {
    hdf5FileMutex = std::make_shared<std::mutex>();
    projectUUID = boost::uuids::random_generator()();
    hdf5FileName = boost::lexical_cast<std::string>(projectUUID) + ".h5";
    hdf5File = std::make_shared<HighFive::File>(hdf5FileName, HighFive::File::ReadWrite | HighFive::File::Create);
  }
  virtual ~FileSetUp()
  {
    std::filesystem::remove(hdf5FileName);
  };
};

TEST_F(FileSetUp, fBwriteLoadTest)
{
  flatbuffers::FlatBufferBuilder fbb(1024);

  // create new Project
  const std::string projectName = "testProject";
  auto seerepCore = std::make_shared<seerep_core::Core>("./");

  seerep_core_msgs::ProjectInfo projectInfo;
  projectInfo.name = projectName;
  projectInfo.uuid = projectUUID;
  seerepCore->newProject(projectInfo);

  // create all nessacary flatbuffer strings
  auto frameId = fbb.CreateString("camera");
  auto encoding = fbb.CreateString("rgb8");
  auto nameOffset = fbb.CreateString(projectName);
  auto uuidOffset = fbb.CreateString(boost::lexical_cast<std::string>(projectUUID));
  uint8_t dataArray[] = { 1, 2, 3, 4 };
  auto data = fbb.CreateVector(dataArray, 4);

  // create timestamp
  seerep::fb::TimestampBuilder timeStampBuilder(fbb);
  timeStampBuilder.add_seconds(std::time(0));
  timeStampBuilder.add_nanos(0);
  flatbuffers::Offset<seerep::fb::Timestamp> timeStampMsg = timeStampBuilder.Finish();
  fbb.Finish(timeStampMsg);

  // create header message
  seerep::fb::HeaderBuilder fbHeaderBuilder(fbb);
  fbHeaderBuilder.add_frame_id(frameId);
  fbHeaderBuilder.add_uuid_project(uuidOffset);
  fbHeaderBuilder.add_stamp(timeStampMsg);
  flatbuffers::Offset<seerep::fb::Header> fbHeaderMsg = fbHeaderBuilder.Finish();
  fbb.Finish(fbHeaderMsg);

  // create image message
  seerep::fb::ImageBuilder fbImgBuilder(fbb);
  fbImgBuilder.add_header(fbHeaderMsg);
  fbImgBuilder.add_height(height);
  fbImgBuilder.add_width(width);
  fbImgBuilder.add_encoding(encoding);
  fbImgBuilder.add_is_bigendian(true);
  fbImgBuilder.add_step(0);
  fbImgBuilder.add_row_step(0);
  fbImgBuilder.add_data(data);
  flatbuffers::Offset<seerep::fb::Image> fbImgMsg = fbImgBuilder.Finish();
  fbb.Finish(fbImgMsg);

  uint8_t* buf = fbb.GetBufferPointer();
  const seerep::fb::Image* writeImgPtr = flatbuffers::GetRoot<seerep::fb::Image>(buf);

  auto imageIo = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(hdf5File, hdf5FileMutex);
  imageIo->writeImage(imgId, *writeImgPtr);

  const seerep::fb::Image* readImgPtr;
  if (auto tmp = imageIo->readImage(imgId))
  {
    readImgPtr = tmp.value().GetRoot();
  };
  fBImageEquals(writeImgPtr, readImgPtr);
  ASSERT_TRUE(fBImageEquals(writeImgPtr, readImgPtr));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
