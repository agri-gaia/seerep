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
to make sure that a Protobuf image message is the same after saving and
reading it from a hdf5 file. Because the underlying components like the
seerep-hdf5-core and seerep-hdf5-pb are not (fully) tested, we don't use mocks and
propagate the data through all components. This should be changed in the future
to improve the quality of the tests.
*/

seerep::Timestamp createTimeStamp(google::protobuf::int32 sec, google::protobuf::uint32 nsec)
{
  seerep::Timestamp ts;
  ts.set_seconds(sec);
  ts.set_nanos(nsec);

  return ts;
}

seerep::Header createHeader(const std::string projectUUID, const std::string messageUUID)
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

seerep::Point2D createPoint(const double x, const double y)
{
  seerep::Point2D ret;
  ret.set_x(x);
  ret.set_y(y);

  return ret;
}

void createImageData(const unsigned int imageHeight, const unsigned int imageWidth, seerep::Image& image)
{
  uint8_t data[imageHeight][imageWidth][3];
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

      data[j][i][0] = r;
      data[j][i][1] = g;
      data[j][i][2] = b;
    }
  }

  image.set_data(data, sizeof(data));
}

seerep::LabelWithInstance createLabelWithInstance()
{
  seerep::LabelWithInstance labelwithinstance;

  boost::uuids::uuid instanceUUID = boost::uuids::random_generator()();
  labelwithinstance.set_label("arbitrary_instance_label");
  labelwithinstance.set_instanceuuid(boost::lexical_cast<std::string>(instanceUUID));

  return labelwithinstance;
}

std::vector<seerep::BoundingBox2DLabeled> createBB2DLabeled(const std::string& projectUUID,
                                                            const std::string& messageUUID)
{
  std::vector<seerep::BoundingBox2DLabeled> bbLabeled;

  for (size_t i = 0; i < 10; i++)
  {
    auto header = createHeader(projectUUID, messageUUID);

    auto pointMin = createPoint(0.01 + i / 10, 0.02 + i / 10);
    auto pointMax = createPoint(0.03 + i / 10, 0.04 + i / 10);

    seerep::Boundingbox2D bb2D;
    bb2D.set_allocated_header(&header);
    bb2D.set_allocated_point_min(&pointMin);
    bb2D.set_allocated_point_max(&pointMin);

    seerep::LabelWithInstance labelWithInstance = createLabelWithInstance();

    seerep::BoundingBox2DLabeled boundingBox2DLabeled;
    boundingBox2DLabeled.set_allocated_labelwithinstance(&labelWithInstance);
    boundingBox2DLabeled.set_allocated_boundingbox(&bb2D);

    bbLabeled.push_back(boundingBox2DLabeled);
  }

  return bbLabeled;
}

auto createImageMessage(const unsigned int imageHeight, const unsigned imageWidth, const std::string& projectUUID,
                        const std::string& messageUUID)
{
  std::string encoding = "rgb8";
  auto header = createHeader(projectUUID, messageUUID);

  // Labels are optional therefore excluded here
  // std::vector<seerep::LabelWithInstance> labelsGeneral;
  // for (size_t i = 0; i < 10; i++)
  // {
  //   auto labelWithInstance = createLabelWithInstance();
  //   labelsGeneral.push_back(labelWithInstance);
  // }
  // auto generalLabels = CreateVector(labelsGeneral.data(), labelsGeneral.size());

  auto bB2DLabeled = createBB2DLabeled(projectUUID, messageUUID);

  seerep::Image imgMsg;
  createImageData(256, 256, imgMsg);

  // set the header
  *imgMsg.mutable_header() = header;
  imgMsg.set_height(imageHeight);
  imgMsg.set_width(imageWidth);
  imgMsg.set_encoding(encoding);
  imgMsg.set_is_bigendian(true);
  imgMsg.set_step(3 * imageHeight);
  imgMsg.set_row_step(0);
  // imgMsg.labels_general = generalLabel;
  // imgMsg.labels_bb = bB2DLabeled;

  // help 1 line
  return imgMsg;
}

class pbWriteLoadTest : public testing::Test
{
protected:
  static std::shared_ptr<std::mutex> hdf5FileMutex;
  static std::string hdf5FileName;
  static std::shared_ptr<HighFive::File> hdf5File;
  static std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> imageIO;

  // static std::string projectUUID;
  static boost::uuids::uuid projectUUID;
  static boost::uuids::uuid messageUUID;
  static std::string projectName;

  static seerep::Image writeImage;
  static seerep::Image readImage;

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

    // temporary files are stored in the build directory of seerep-hdf5-pb
    // auto seerepCore = std::make_shared<seerep_core::Core>("./", false);
    // seerep_core_msgs::ProjectInfo projectInfo;
    // projectInfo.name = projectName;
    // projectInfo.uuid = projectUUID;
    // seerepCore->createProject(projectInfo);

    imageIO = std::make_shared<seerep_hdf5_pb::Hdf5PbImage>(hdf5File, hdf5FileMutex);
    if (imageIO == nullptr)
    {
      GTEST_FATAL_FAILURE_("Error: Can't create HDF5Image object for writing images");
    }

    writeImage = createImageMessage(256, 256, boost::lexical_cast<std::string>(projectUUID),
                                    boost::lexical_cast<std::string>(messageUUID));
    // if (writeImage == NULL)
    // {
    //   GTEST_FATAL_FAILURE_("Error: No image data to write into HDF5 file");
    // }

    imageIO->writeImage(boost::lexical_cast<std::string>(messageUUID), writeImage);

    auto gRPCImage = imageIO->readImage(boost::lexical_cast<std::string>(messageUUID));

    if (!gRPCImage.has_value())
    {
      GTEST_FATAL_FAILURE_("Error: No data could be read from HDF5 file");
    };

    readImage = gRPCImage.value();
  }

  static void TearDownTestSuite()
  {
    std::filesystem::remove(hdf5FileName);
  }
};

std::shared_ptr<std::mutex> pbWriteLoadTest::hdf5FileMutex;
std::shared_ptr<HighFive::File> pbWriteLoadTest::hdf5File;
std::string pbWriteLoadTest::hdf5FileName;

boost::uuids::uuid pbWriteLoadTest::projectUUID;
boost::uuids::uuid pbWriteLoadTest::messageUUID;
std::string pbWriteLoadTest::projectName;

std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> pbWriteLoadTest::imageIO;

seerep::Image pbWriteLoadTest::writeImage;
seerep::Image pbWriteLoadTest::readImage;

TEST_F(pbWriteLoadTest, testImageHeader)
{
  EXPECT_EQ(readImage.header().stamp().seconds(), writeImage.header().stamp().seconds());
  EXPECT_EQ(readImage.header().stamp().nanos(), writeImage.header().stamp().nanos());
  EXPECT_STREQ(readImage.header().frame_id().c_str(), writeImage.header().frame_id().c_str());
  EXPECT_STREQ(readImage.header().uuid_project().c_str(), writeImage.header().uuid_project().c_str());
  EXPECT_STREQ(readImage.header().uuid_msgs().c_str(), writeImage.header().uuid_msgs().c_str());
}

TEST_F(pbWriteLoadTest, testImageBaseFields)
{
  EXPECT_EQ(readImage.height(), writeImage.height());
  EXPECT_EQ(readImage.width(), writeImage.width());
  EXPECT_STREQ(readImage.encoding().c_str(), writeImage.encoding().c_str());
  EXPECT_EQ(readImage.is_bigendian(), writeImage.is_bigendian());
  EXPECT_EQ(readImage.step(), writeImage.step());
  EXPECT_EQ(readImage.row_step(), writeImage.row_step());
}

TEST_F(pbWriteLoadTest, testImageData)
{
  ASSERT_EQ(readImage.data().size(), writeImage.data().size());
  for (size_t i = 0; i < readImage.data().size(); i++)
  {
    EXPECT_EQ(readImage.data()[i], writeImage.data()[i]);
  }
}

void testLabelWithInstance(const seerep::LabelWithInstance* readInstance, const seerep::LabelWithInstance* writeInstance)
{
  if (readInstance == nullptr || writeInstance == nullptr)
  {
    FAIL() << "Error: Can't compare a LabelWithInstance to nullptr";
  }
  EXPECT_STREQ(readInstance->label().c_str(), writeInstance->label().c_str());
  EXPECT_STREQ(readInstance->instanceuuid().c_str(), writeInstance->instanceuuid().c_str());
}

TEST_F(pbWriteLoadTest, testGeneralLabels)
{
  ASSERT_EQ(readImage.labels_general().size(), writeImage.labels_general().size());
  for (int i = 0; i < readImage.labels_general().size(); i++)
  {
    testLabelWithInstance(&readImage.labels_general().Get(i), &writeImage.labels_general().Get(i));
  }
}

void testEqualPoints(const seerep::Point2D* readPoint, const seerep::Point2D* writePoint)
{
  if (readPoint == nullptr || writePoint == nullptr)
  {
    FAIL() << "Error: Can't compare a point to nullptr";
  }
  EXPECT_EQ(readPoint->x(), writePoint->x());
  EXPECT_EQ(readPoint->y(), writePoint->y());
}

// // don't test the header since, it will be removed in #72
// TEST_F(pbWriteLoadTest, testBoundingBox2DLabeled)
// {
//   ASSERT_EQ(readImage->labels_bb().size(), writeImage->labels_bb().size());
//   for (size_t i = 0; i < readImage->labels_bb().size(); i++)
//   {
//     testLabelWithInstance(readImage->labels_bb().Get(i).labelwithinstance(),
//                           writeImage->labels_bb().Get(i).labelwithinstance
//     testEqualPoints(readImage->labels_bb().Get(i).boundingbox().point_min(),
//                     writeImage->labels_bb().Get(i).boundingbox().point_min());
//     testEqualPoints(readImage->labels_bb().Get(i).boundingbox().point_max(),
//                     writeImage->labels_bb().Get(i).boundingbox().point_max());
//   }
// }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
