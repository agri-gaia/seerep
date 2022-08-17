#include <gtest/gtest.h>
#include <seerep-core/core.h>
#include <seerep-hdf5-pb/hdf5-pb-image.h>

#include <H5File.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <ctime>
#include <optional>
#include <string>
#include <vector>

/*
This test serves more as an integration test rather than a unit test. We want
to make sure that a Flabuffers image message is the same after saving and
reading it from a hdf5 file. Because the underlying components like the
seerep-hdf5-core and seerep-hdf5-fb are not (fully) tested, we don't use mocks and
propagate the data through all components. This should be changed in the future
to improve the quality of the tests.
*/

/*
 */

auto createTimeStamp(google::protobuf::int32 sec, google::protobuf::uint32 nsec)
{
  seerep::Timestamp ts;
  ts.set_seconds(sec);
  ts.set_nanos(nsec);

  return ts;
}

auto createHeader(const std::string projectUUID, const std::string messageUUID)
{
  seerep::Header ret;

  ret.set_seq(5);
  ret.set_frame_id("arbitrary_id");
  ret.mutable_stamp()->set_seconds(5);
  ret.mutable_stamp()->set_nanos(5);
  ret.set_uuid_project(projectUUID);
  ret.set_uuid_msgs(messageUUID);

  return ret;
}

auto createPoint(const double x, const double y)
{
  seerep::Point ret;
  ret.set_x(x);
  ret.set_y(y);

  return ret;
}

auto createImageData(const unsigned int imageHeight, const unsigned int imageWidth)
{
  std::vector<u_int8_t> data;
  for (size_t i = 0; i < imageWidth; i++)
  {
    for (size_t j = 0; j < imageHeight; j++)
    {
      float x = float(i) / imageHeight;
      float y = float(j) / imageHeight;
      float z = float(j) / imageHeight;

      uint8_t r = int((x * 255.0)) % 255;
      uint8_t g = int((y * 255.0)) % 255;
      uint8_t b = int((z * 255.0)) % 255;

      data.push_back(r);
      data.push_back(g);
      data.push_back(b);
    }
  }

  seerep::Image ret;
  ret.set_width(imageWidth);
  ret.set_height(imageHeight);
  ret.set_data(&data.front(), data.size());

  return ret;
}

auto createLabelWithInstance()
{
  seerep::LabelWithInstance lwt;
  lwt.set_label("arbitrary_instance_label");
  lwt.set_instanceuuid("arbitrary_instance_uuid");

  return lwt;
}

auto createBB2DLabeled(const std::string& projectUUID, const std::string& messageUUID)
{
  std::vector<seerep::BoundingBox2DLabeled> bbLabeled;
  for (size_t i = 0; i < 10; i++)
  {
    auto headerOffset = createHeader(projectUUID, messageUUID);

    auto pointMinOffset = createPoint(0.01 + i / 10, 0.02 + i / 10);
    auto pointMaxOffset = createPoint(0.03 + i / 10, 0.04 + i / 10);

    seerep::Boundingbox2D bb2DOffset;
    // help 3 lines
    bb2DOffset.set_header(headerOffset);
    bb2DOffset.set_point_min(pointMinOffset);
    bb2DOffset.set_point_max(pointMaxOffset);

    auto labelWithInstanceOffset = createLabelWithInstance();

    seerep::BoundingBox2DLabeled boundingBox2DLabeledOffset;
    // help 2 lines
    boundingBox2DLabeledOffset.labelwithinstance = labelWithInstanceOffset;
    boundingBox2DLabeledOffset.boundingBox = bb2DOffset;

    bbLabeled.push_back(boundingBox2DLabeledOffset);
  }

  return bbLabeled;
}

auto createImageMessage(const unsigned int imageHeight, const unsigned imageWidth, const std::string& projectUUID,
                        const std::string& messageUUID)
{
  std::string encodingOffset = "rgb8";
  auto headerOffset = createHeader(projectUUID, messageUUID);
  auto imageOffset = createImageData(256, 256);

  // Labels are optional therefore excluded here
  // std::vector<seerep::LabelWithInstance> labelsGeneral;
  // for (size_t i = 0; i < 10; i++)
  // {
  //   auto labelWithInstanceOffset = createLabelWithInstance();
  //   labelsGeneral.push_back(labelWithInstanceOffset);
  // }
  // auto generalLabelsOffset = CreateVector(labelsGeneral.data(), labelsGeneral.size());

  auto bB2DLabeledOffset = createBB2DLabeled(projectUUID, messageUUID);

  seerep::Image imgMsgOffset;

  // set the header
  *imgMsgOffset.mutable_header() = headerOffset;
  imgMsgOffset.set_height(imageHeight);
  imgMsgOffset.set_width(imageWidth);
  imgMsgOffset.set_encoding(encodingOffset);
  imgMsgOffset.set_is_bigendian(true);
  imgMsgOffset.set_step(3 * imageHeight);
  imgMsgOffset.set_row_step(0);
  // help 1 line
  imgMsgOffset.set_data(&imageOffset.data.front(), imageOffset.data.size());
  // imgMsgOffset.labels_general = generalLabelOffset;
  // imgMsgOffset.labels_bb = bB2DLabeledOffset;

  // help 1 line
  return google::protobuf::GetRoot<seerep::Image>(imgMsgOffset);
}

/*
class fbWriteLoadTest : public testing::Test
{
protected:
  static std::shared_ptr<std::mutex> hdf5FileMutex;
  static std::string hdf5FileName;
  static std::shared_ptr<HighFive::File> hdf5File;
  static std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> imageIO;

  static boost::uuids::uuid projectUUID;
  static boost::uuids::uuid messageUUID;
  static std::string projectName;

  // Because the tests only compare the written and read data, we create the
  // resources only once and share them between the tests
  static void SetUpTestSuite()
  {
    projectName = "testProject";
    projectUUID = boost::uuids::random_generator()();
    messageUUID = boost::uuids::random_generator()();

    hdf5FileMutex = std::make_shared<std::mutex>();
    hdf5FileName = boost::lexical_cast<std::string>(projectUUID) + ".h5";
    hdf5File = std::make_shared<HighFive::File>(hdf5FileName, HighFive::File::ReadWrite | HighFive::File::Create);

    // temporary files are stored in the build directory of seerep-hdf5-fb
    auto seerepCore = std::make_shared<seerep_core::Core>("./", false);
    seerep_core_msgs::ProjectInfo projectInfo;
    projectInfo.name = projectName;
    projectInfo.uuid = projectUUID;
    seerepCore->createProject(projectInfo);

    imageIO = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(hdf5File, hdf5FileMutex);
    if (imageIO == nullptr)
    {
      GTEST_FATAL_FAILURE_("Error: Can't create HDF5Image object for writing images");
    }

    writeImage = createImageMessage(256, 256, boost::lexical_cast<std::string>(projectUUID),
                                    boost::lexical_cast<std::string>(messageUUID));
    if (writeImage == nullptr)
    {
      GTEST_FATAL_FAILURE_("Error: No image data to write into HDF5 file");
    }

    imageIO->writeImage(boost::lexical_cast<std::string>(messageUUID), *writeImage);

    auto gRPCImage = imageIO->readImage(boost::lexical_cast<std::string>(messageUUID));
    if (!gRPCImage.has_value())
    {
      GTEST_FATAL_FAILURE_("Error: No data could be read from HDF5 file");
    };

    readImage = gRPCImage.value().GetRoot();
  }

  static void TearDownTestSuite()
  {
    std::filesystem::remove(hdf5FileName);
  }
};

std::shared_ptr<std::mutex> fbWriteLoadTest::hdf5FileMutex;
std::shared_ptr<HighFive::File> fbWriteLoadTest::hdf5File;
std::string fbWriteLoadTest::hdf5FileName;

boost::uuids::uuid fbWriteLoadTest::projectUUID;
boost::uuids::uuid fbWriteLoadTest::messageUUID;
std::string fbWriteLoadTest::projectName;

std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> fbWriteLoadTest::imageIO;

const seerep::fb::Image* fbWriteLoadTest::writeImage;
const seerep::fb::Image* fbWriteLoadTest::readImage;

TEST_F(fbWriteLoadTest, testImageHeader)
{
  EXPECT_EQ(readImage->header()->stamp()->seconds(), writeImage->header()->stamp()->seconds());
  EXPECT_EQ(readImage->header()->stamp()->nanos(), writeImage->header()->stamp()->nanos());
  EXPECT_STREQ(readImage->header()->frame_id()->c_str(), writeImage->header()->frame_id()->c_str());
  EXPECT_STREQ(readImage->header()->uuid_project()->c_str(), writeImage->header()->uuid_project()->c_str());
  EXPECT_STREQ(readImage->header()->uuid_msgs()->c_str(), writeImage->header()->uuid_msgs()->c_str());
}

TEST_F(fbWriteLoadTest, testImageBaseFields)
{
  EXPECT_EQ(readImage->height(), writeImage->height());
  EXPECT_EQ(readImage->width(), writeImage->width());
  EXPECT_STREQ(readImage->encoding()->c_str(), writeImage->encoding()->c_str());
  EXPECT_EQ(readImage->is_bigendian(), writeImage->is_bigendian());
  EXPECT_EQ(readImage->step(), writeImage->step());
  EXPECT_EQ(readImage->row_step(), writeImage->row_step());
}

TEST_F(fbWriteLoadTest, testImageData)
{
  ASSERT_EQ(readImage->data()->size(), writeImage->data()->size());
  for (size_t i = 0; i < readImage->data()->size(); i++)
  {
    EXPECT_EQ(readImage->data()->Get(i), writeImage->data()->Get(i));
  }
}

void testLabelWithInstance(const seerep::fb::LabelWithInstance* readInstance,
                           const seerep::fb::LabelWithInstance* writeInstance)
{
  if (readInstance == nullptr || writeInstance == nullptr)
  {
    FAIL() << "Error: Can't compare a LabelWithInstance to nullptr";
  }
  EXPECT_STREQ(readInstance->label()->c_str(), writeInstance->label()->c_str());
  EXPECT_STREQ(readInstance->instanceUuid()->c_str(), writeInstance->instanceUuid()->c_str());
}

TEST_F(fbWriteLoadTest, testGeneralLabels)
{
  ASSERT_EQ(readImage->labels_general()->size(), writeImage->labels_general()->size());
  for (size_t i = 0; i < readImage->labels_general()->size(); i++)
  {
    testLabelWithInstance(readImage->labels_general()->Get(i), writeImage->labels_general()->Get(i));
  }
}

void testEqualPoints(const seerep::fb::Point2D* readPoint, const seerep::fb::Point2D* writePoint)
{
  if (readPoint == nullptr || writePoint == nullptr)
  {
    FAIL() << "Error: Can't compare a point to nullptr";
  }
  EXPECT_EQ(readPoint->x(), writePoint->x());
  EXPECT_EQ(readPoint->y(), writePoint->y());
}

// don't test the header since, it will be removed in #72
TEST_F(fbWriteLoadTest, testBoundingBox2DLabeled)
{
  ASSERT_EQ(readImage->labels_bb()->size(), writeImage->labels_bb()->size());
  for (size_t i = 0; i < readImage->labels_bb()->size(); i++)
  {
    testLabelWithInstance(readImage->labels_bb()->Get(i)->labelWithInstance(),
                          writeImage->labels_bb()->Get(i)->labelWithInstance());
    testEqualPoints(readImage->labels_bb()->Get(i)->bounding_box()->point_min(),
                    writeImage->labels_bb()->Get(i)->bounding_box()->point_min());
    testEqualPoints(readImage->labels_bb()->Get(i)->bounding_box()->point_max(),
                    writeImage->labels_bb()->Get(i)->bounding_box()->point_max());
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
*/
