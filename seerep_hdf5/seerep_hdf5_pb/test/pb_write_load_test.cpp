#include <gtest/gtest.h>

#include <H5File.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <ctime>
#include <optional>
#include <string>
#include <vector>

#include "seerep_hdf5_pb/hdf5_pb_image.h"

/*
This test serves more as an integration test rather than a unit test. We want
to make sure that a Protobuf image message is the same after saving and
reading it from a hdf5 file. Because the underlying components like the
seerep-hdf5-core and seerep-hdf5-pb are not (fully) tested, we don't use mocks
and propagate the data through all components. This should be changed in the
future to improve the quality of the tests.
*/

/**
 * @brief given a pointer to an image header, sets sub field values.
 * @param[in] projectUUID the UUID of the project
 * @param[in] messageUUId the UUID of the message
 * @param[in,out] header reference to the header which will hold the fields
 **/
void createHeader(const std::string projectUUID, const std::string messageUUID,
                  seerep::pb::Header& header)
{
  header.set_seq(5);
  header.set_frame_id("arbitrary_id");
  header.mutable_stamp()->set_seconds(5);
  header.mutable_stamp()->set_nanos(6);
  header.set_uuid_project(projectUUID);
  header.set_uuid_msgs(messageUUID);
}

/**
 * @brief creates a grid of image data given height and width of an image
 * @param[in] imageHeight the height of the image
 * @param[in] imageWidth the width of the image
 * @param[in,out] image reference to the image object
 * */
void createImageData(const unsigned int imageHeight,
                     const unsigned int imageWidth, seerep::pb::Image& image)
{
  std::vector<uint8_t> data;
  data.reserve(imageHeight * imageWidth * 3);
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

  *image.mutable_data() = { data.begin(), data.end() };
}

/**
 * @brief given a label set arbitrary label and uuid
 * @param[in,out] label a pointer to a label with instance
 * */
void createLabel(seerep::pb::Label& label)
{
  boost::uuids::uuid instanceUUID = boost::uuids::random_generator()();
  label.set_label("arbitrary_instance_label");
  label.set_labeliddatumaro(42);
  label.set_instanceuuid(boost::lexical_cast<std::string>(instanceUUID));
  label.set_instanceiddatumaro(43);
}

/**
 * @brief create labels general
 * @param[in] image reference to the image
 * */
void createLabelsGeneral(seerep::pb::Image& image)
{
  for (size_t iCategory = 0; iCategory < 3; iCategory++)
  {
    auto lables = image.add_labels();
    lables->set_category("category" + std::to_string(iCategory));
    lables->set_datumarojson("random string to test datumaro field");
    for (size_t i = 0; i < 10; i++)
    {
      createLabel(*lables->add_labels());
    }
  }
}

/**
 * @brief give image height and width, and project and message uuid, build and
 * return an image
 * @param[in] imageHeight the height of the image
 * @param[in] imageWidth the width of the image
 * @param[in] projectUUID the uuid of the project
 * @param[in] messageUUID the uuid of the message
 * @return seerep:Image object
 * */
seerep::pb::Image createImageMessage(const unsigned int imageHeight,
                                     const unsigned imageWidth,
                                     const std::string& projectUUID,
                                     const std::string& messageUUID,
                                     const std::string& cameraintrinsicsUUID)
{
  std::string encoding = "rgb8";

  seerep::pb::Image imgMsg;
  createHeader(projectUUID, messageUUID, *imgMsg.mutable_header());
  createImageData(256, 256, imgMsg);

  // set the header
  imgMsg.set_height(imageHeight);
  imgMsg.set_width(imageWidth);
  imgMsg.set_encoding(encoding);
  imgMsg.set_is_bigendian(true);
  imgMsg.set_step(3 * imageHeight);
  imgMsg.set_row_step(0);
  imgMsg.set_uuid_camera_intrinsics(cameraintrinsicsUUID);

  createLabelsGeneral(imgMsg);

  return imgMsg;
}

/**
 * @brief class holds all the data structures which will be tested.
 * */
class pbWriteLoadTest : public testing::Test
{
protected:
  static std::shared_ptr<std::mutex> hdf5FileMutex;
  static std::string hdf5FileName;
  static std::shared_ptr<HighFive::File> hdf5File;
  static std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> imageIO;

  static boost::uuids::uuid projectUUID;
  static boost::uuids::uuid messageUUID;
  static boost::uuids::uuid cameraintrinsicsUUID;
  static std::string projectName;

  static seerep::pb::Image writeImage;
  static seerep::pb::Image readImage;

  // Because the tests only compare the written and read data, we create the
  // resources only once and share them between the tests
  /**
   * @brief This function initializes the data structures to be tested and
   * populates them with arbitrary values.
   * */
  static void SetUpTestSuite()
  {
    projectName = "testProject";
    projectUUID = boost::uuids::random_generator()();
    messageUUID = boost::uuids::random_generator()();
    cameraintrinsicsUUID = boost::uuids::random_generator()();

    hdf5FileMutex = std::make_shared<std::mutex>();
    hdf5FileName = boost::lexical_cast<std::string>(projectUUID) + ".h5";
    hdf5File = std::make_shared<HighFive::File>(
        hdf5FileName, HighFive::File::ReadWrite | HighFive::File::Create);

    imageIO =
        std::make_shared<seerep_hdf5_pb::Hdf5PbImage>(hdf5File, hdf5FileMutex);
    if (imageIO == nullptr)
    {
      GTEST_FATAL_FAILURE_(
          "Error: Can't create HDF5Image object for writing images");
    }

    writeImage = createImageMessage(
        256, 256, boost::lexical_cast<std::string>(projectUUID),
        boost::lexical_cast<std::string>(messageUUID),
        boost::lexical_cast<std::string>(cameraintrinsicsUUID));

    imageIO->writeImage(boost::lexical_cast<std::string>(messageUUID),
                        writeImage);

    auto gRPCImage =
        imageIO->readImage(boost::lexical_cast<std::string>(messageUUID));

    if (!gRPCImage.has_value())
    {
      GTEST_FATAL_FAILURE_("Error: No data could be read from HDF5 file");
    };

    readImage = gRPCImage.value();
  }

  /**
   * @brief Destroy the hdf5 file created for the tests.
   * */
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
boost::uuids::uuid pbWriteLoadTest::cameraintrinsicsUUID;
std::string pbWriteLoadTest::projectName;

std::shared_ptr<seerep_hdf5_pb::Hdf5PbImage> pbWriteLoadTest::imageIO;

seerep::pb::Image pbWriteLoadTest::writeImage;
seerep::pb::Image pbWriteLoadTest::readImage;

/**
 * @brief test original header and converted header read from hdf5 file for equality.
 * @param[in] pbWriteLoadTest test suite class
 * @param testImageHeader name of test
 * */
TEST_F(pbWriteLoadTest, testImageHeader)
{
  EXPECT_EQ(readImage.header().stamp().seconds(),
            writeImage.header().stamp().seconds());
  EXPECT_EQ(readImage.header().stamp().nanos(),
            writeImage.header().stamp().nanos());
  EXPECT_STREQ(readImage.header().frame_id().c_str(),
               writeImage.header().frame_id().c_str());
  EXPECT_STREQ(readImage.header().uuid_project().c_str(),
               writeImage.header().uuid_project().c_str());
  EXPECT_STREQ(readImage.header().uuid_msgs().c_str(),
               writeImage.header().uuid_msgs().c_str());
}

/**
 * @brief test original image base fields and converted image base fields read
 * from hdf5 file for equality.
 * @param[in] pbWriteLoadTest test suite class
 * @param testImageBaseFields name of test
 * */
TEST_F(pbWriteLoadTest, testImageBaseFields)
{
  EXPECT_EQ(readImage.height(), writeImage.height());
  EXPECT_EQ(readImage.width(), writeImage.width());
  EXPECT_STREQ(readImage.encoding().c_str(), writeImage.encoding().c_str());
  EXPECT_EQ(readImage.is_bigendian(), writeImage.is_bigendian());
  EXPECT_EQ(readImage.step(), writeImage.step());
  EXPECT_EQ(readImage.row_step(), writeImage.row_step());
  EXPECT_EQ(readImage.uuid_camera_intrinsics(),
            writeImage.uuid_camera_intrinsics());
}

/**
 * @brief test original image data and converted image data read from hdf5 file
 * for equality.
 * @param[in] pbWriteLoadTest test suite class
 * @param testImageData name of test
 * */
TEST_F(pbWriteLoadTest, testImageData)
{
  ASSERT_EQ(readImage.data().size(), writeImage.data().size());
  for (size_t i = 0; i < readImage.data().size(); i++)
  {
    EXPECT_EQ(readImage.data()[i], writeImage.data()[i]);
  }
}

/**
 * @brief test original label with instance and converted label with instance
 * read from hdf5 file for equality.
 * @param[in] readInstance the Label which was read
 * @param[in] writeInstance the Label which was written
 * */
void testLabelWithInstance(const seerep::pb::Label& readInstance,
                           const seerep::pb::Label& writeInstance)
{
  EXPECT_STREQ(readInstance.label().c_str(), writeInstance.label().c_str());
  EXPECT_EQ(readInstance.labeliddatumaro(), writeInstance.labeliddatumaro());
  EXPECT_STREQ(readInstance.instanceuuid().c_str(),
               writeInstance.instanceuuid().c_str());
  EXPECT_EQ(readInstance.instanceiddatumaro(),
            writeInstance.instanceiddatumaro());
}

void testLabelsWithInstanceWithCategory(
    const seerep::pb::LabelCategory& readInstance,
    const seerep::pb::LabelCategory& writeInstance)
{
  EXPECT_STREQ(readInstance.category().c_str(),
               writeInstance.category().c_str());
  for (int i = 0; i < readInstance.labels_size(); i++)
  {
    testLabelWithInstance(readInstance.labels().Get(i),
                          writeInstance.labels().Get(i));
  }
}

/**
 * @brief test original general labels and converted general label read from
 * hdf5 file for equality.
 * @param[in] pbWriteLoadTest test suite class
 * @param testGeneralLabels name of test
 * */
TEST_F(pbWriteLoadTest, testGeneralLabels)
{
  ASSERT_EQ(readImage.labels().size(), writeImage.labels().size());
  for (int i = 0; i < readImage.labels().size(); i++)
  {
    testLabelsWithInstanceWithCategory(readImage.labels().Get(i),
                                       writeImage.labels().Get(i));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
