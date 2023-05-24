#include <gtest/gtest.h>

#include <H5File.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <ctime>
#include <optional>
#include <string>
#include <vector>

#include "seerep_hdf5_fb/hdf5_fb_image.h"

/*
This test serves more as an integration test rather than a unit test. We want
to make sure that a Flatbuffers image message is the same after saving and
reading it from a hdf5 file. Because the underlying components like the
seerep-hdf5-core and seerep-hdf5-fb are not (fully) tested, we don't use mocks and
propagate the data through all components. This should be changed in the future
to improve the quality of the tests.
*/

namespace seerep_hdf5_fb
{
namespace tests
{
flatbuffers::Offset<seerep::fb::Timestamp> createTimeStamp(flatbuffers::FlatBufferBuilder& fbb)
{
  auto timeStampMsgOffset = seerep::fb::CreateTimestamp(fbb, std::time(0), 0);
  fbb.Finish(timeStampMsgOffset);
  return timeStampMsgOffset;
}

flatbuffers::Offset<seerep::fb::Header> createHeader(flatbuffers::FlatBufferBuilder& fbb, const std::string& frameId,
                                                     const std::string& projectUUID, const std::string& messageUUID)
{
  auto frameIdOffset = fbb.CreateString(frameId);
  auto projectUUIDOffset = fbb.CreateString(projectUUID);
  auto messageUUIDOffset = fbb.CreateString(messageUUID);
  auto timeStampOffset = createTimeStamp(fbb);
  auto headerMsgOffset =
      seerep::fb::CreateHeader(fbb, 0, timeStampOffset, frameIdOffset, projectUUIDOffset, messageUUIDOffset);
  // uint8_t* buf = fbb.GetBufferPointer();
  return headerMsgOffset;
}

flatbuffers::Offset<seerep::fb::Point2D> createPoint(flatbuffers::FlatBufferBuilder& fbb, const double x, const double y)
{
  auto pointOffset = seerep::fb::CreatePoint2D(fbb, x, y);
  fbb.Finish(pointOffset);
  return pointOffset;
}

flatbuffers::Offset<flatbuffers::Vector<uint8_t>>
createImageData(flatbuffers::FlatBufferBuilder& fbb, const unsigned int imageHeight, const unsigned int imageWidth)
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

  auto fbImageDataOffset = fbb.CreateVector(data.data(), data.size());
  fbb.Finish(fbImageDataOffset);
  return fbImageDataOffset;
}

flatbuffers::Offset<seerep::fb::LabelWithInstance> createLabelWithInstance(flatbuffers::FlatBufferBuilder& fbb)
{
  boost::uuids::uuid instanceUUID = boost::uuids::random_generator()();
  auto instanceUUIDOffset = fbb.CreateString(boost::lexical_cast<std::string>(instanceUUID));
  auto labelStr = fbb.CreateString("testLabelGeneral");

  auto label = seerep::fb::CreateLabel(fbb, labelStr, 0.5);

  auto labelWithInstanceOffset = seerep::fb::CreateLabelWithInstance(fbb, label, instanceUUIDOffset);
  fbb.Finish(labelWithInstanceOffset);
  return labelWithInstanceOffset;
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>>>
createBB2DLabeled(flatbuffers::FlatBufferBuilder& fbb)
{
  std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeledWithCategory>> bbCategory;
  for (size_t iCategory = 0; iCategory < 3; iCategory++)
  {
    auto categoryOffset = fbb.CreateString("category" + std::to_string(iCategory));

    std::vector<flatbuffers::Offset<seerep::fb::BoundingBox2DLabeled>> bbLabeled;
    for (size_t i = 0; i < 10; i++)
    {
      auto pointMinOffset = createPoint(fbb, 0.01 + i / 10, 0.02 + i / 10);
      auto pointMaxOffset = createPoint(fbb, 0.03 + i / 10, 0.04 + i / 10);

      auto bb2DOffset = seerep::fb::CreateBoundingbox2D(fbb, pointMinOffset, pointMaxOffset);
      fbb.Finish(bb2DOffset);

      auto labelWithInstanceOffset = createLabelWithInstance(fbb);

      seerep::fb::BoundingBox2DLabeledBuilder bbLabeledBuilder(fbb);
      bbLabeledBuilder.add_bounding_box(bb2DOffset);
      bbLabeledBuilder.add_labelWithInstance(labelWithInstanceOffset);
      bbLabeled.push_back(bbLabeledBuilder.Finish());
    }

    auto bb2dLabeledOffset = fbb.CreateVector(bbLabeled);
    seerep::fb::BoundingBox2DLabeledWithCategoryBuilder bbCategoryBuilder(fbb);
    bbCategoryBuilder.add_category(categoryOffset);
    bbCategoryBuilder.add_boundingBox2dLabeled(bb2dLabeledOffset);
    bbCategory.push_back(bbCategoryBuilder.Finish());
  }
  auto labelsBBOffset = fbb.CreateVector(bbCategory);
  fbb.Finish(labelsBBOffset);
  return labelsBBOffset;
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>>>
createLabelsGeneral(flatbuffers::FlatBufferBuilder& fbb)
{
  std::vector<flatbuffers::Offset<seerep::fb::LabelsWithInstanceWithCategory>> labelsCategories;
  for (size_t iCategory = 0; iCategory < 3; iCategory++)
  {
    auto categoryOffset = fbb.CreateString("category" + std::to_string(iCategory));

    std::vector<flatbuffers::Offset<seerep::fb::LabelWithInstance>> labelsWithInstanceVector;
    for (size_t i = 0; i < 10; i++)
    {
      labelsWithInstanceVector.push_back(createLabelWithInstance(fbb));
    }

    auto labelWithInstanceOffset = fbb.CreateVector(labelsWithInstanceVector);

    seerep::fb::LabelsWithInstanceWithCategoryBuilder labelsCategoriesBuilder(fbb);
    labelsCategoriesBuilder.add_category(categoryOffset);
    labelsCategoriesBuilder.add_labelsWithInstance(labelWithInstanceOffset);
    labelsCategories.push_back(labelsCategoriesBuilder.Finish());
  }
  auto labelsCategoriesOffset = fbb.CreateVector(labelsCategories);
  fbb.Finish(labelsCategoriesOffset);
  return labelsCategoriesOffset;
}

const seerep::fb::Image* createImageMessage(flatbuffers::FlatBufferBuilder& fbb, const unsigned int imageHeight,
                                            const unsigned imageWidth, const std::string& projectUUID,
                                            const std::string& messageUUID, const std::string& camintrinsicsUUID)
{
  auto encodingOffset = fbb.CreateString("rgb8");
  auto camintrinsicsUUIDOffset = fbb.CreateString(camintrinsicsUUID);

  auto headerOffset = createHeader(fbb, "camera", projectUUID, messageUUID);
  auto imageOffset = createImageData(fbb, 256, 256);

  auto generalLabelsOffset = createLabelsGeneral(fbb);
  auto bB2DLabeledOffset = createBB2DLabeled(fbb);

  auto imgMsgOffset =
      seerep::fb::CreateImage(fbb, headerOffset, imageHeight, imageWidth, encodingOffset, true, 3 * imageHeight,
                              imageOffset, generalLabelsOffset, bB2DLabeledOffset, camintrinsicsUUIDOffset);
  fbb.Finish(imgMsgOffset);
  uint8_t* buf = fbb.GetBufferPointer();
  return flatbuffers::GetRoot<seerep::fb::Image>(buf);
}

class fbWriteLoadTest : public testing::Test
{
protected:
  static std::shared_ptr<std::mutex> hdf5FileMutex;
  static std::string hdf5FileName;
  static std::shared_ptr<HighFive::File> hdf5File;
  static std::shared_ptr<seerep_hdf5_fb::Hdf5FbImage> imageIO;

  static boost::uuids::uuid projectUUID;
  static boost::uuids::uuid messageUUID;
  static boost::uuids::uuid cameraintrinsicsUUID;
  static std::string projectName;

  static const seerep::fb::Image* writeImage;
  static const seerep::fb::Image* readImage;

  static flatbuffers::FlatBufferBuilder fbb;

  // Because the tests only compare the written and read data, we create the
  // resources only once and share them between the tests
  static void SetUpTestSuite()
  {
    projectName = "testProject";
    projectUUID = boost::uuids::random_generator()();
    messageUUID = boost::uuids::random_generator()();
    cameraintrinsicsUUID = boost::uuids::random_generator()();

    hdf5FileMutex = std::make_shared<std::mutex>();
    hdf5FileName = boost::lexical_cast<std::string>(projectUUID) + ".h5";
    hdf5File = std::make_shared<HighFive::File>(hdf5FileName, HighFive::File::ReadWrite | HighFive::File::Create);

    fbb = flatbuffers::FlatBufferBuilder(1024);

    imageIO = std::make_shared<seerep_hdf5_fb::Hdf5FbImage>(hdf5File, hdf5FileMutex);
    if (imageIO == nullptr)
    {
      GTEST_FATAL_FAILURE_("Error: Can't create HDF5Image object for writing images");
    }

    writeImage = createImageMessage(fbb, 256, 256, boost::lexical_cast<std::string>(projectUUID),
                                    boost::lexical_cast<std::string>(messageUUID),
                                    boost::lexical_cast<std::string>(cameraintrinsicsUUID));
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
boost::uuids::uuid fbWriteLoadTest::cameraintrinsicsUUID;
std::string fbWriteLoadTest::projectName;

flatbuffers::FlatBufferBuilder fbWriteLoadTest::fbb;
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
  EXPECT_EQ(readImage->uuid_cameraintrinsics()->str(), writeImage->uuid_cameraintrinsics()->str());
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
  EXPECT_STREQ(readInstance->label()->label()->c_str(), writeInstance->label()->label()->c_str());
  EXPECT_FLOAT_EQ(readInstance->label()->confidence(), writeInstance->label()->confidence());
  EXPECT_STREQ(readInstance->instanceUuid()->c_str(), writeInstance->instanceUuid()->c_str());
}

void testLabelWithInstanceCategories(const seerep::fb::LabelsWithInstanceWithCategory* readInstance,
                                     const seerep::fb::LabelsWithInstanceWithCategory* writeInstance)
{
  if (readInstance == nullptr || writeInstance == nullptr)
  {
    FAIL() << "Error: Can't compare a LabelWithInstance to nullptr";
  }
  EXPECT_STREQ(readInstance->category()->c_str(), writeInstance->category()->c_str());
  for (size_t i = 0; i < readInstance->labelsWithInstance()->size(); i++)
  {
    testLabelWithInstance(readInstance->labelsWithInstance()->Get(i), writeInstance->labelsWithInstance()->Get(i));
  }
}

TEST_F(fbWriteLoadTest, testGeneralLabels)
{
  ASSERT_EQ(readImage->labels_general()->size(), writeImage->labels_general()->size());
  for (size_t i = 0; i < readImage->labels_general()->size(); i++)
  {
    testLabelWithInstanceCategories(readImage->labels_general()->Get(i), writeImage->labels_general()->Get(i));
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

void testBB2DWithInstance(const seerep::fb::BoundingBox2DLabeled* readInstance,
                          const seerep::fb::BoundingBox2DLabeled* writeInstance)
{
  if (readInstance == nullptr || writeInstance == nullptr)
  {
    FAIL() << "Error: Can't compare a LabelWithInstance to nullptr";
  }
  EXPECT_STREQ(readInstance->labelWithInstance()->label()->label()->c_str(),
               writeInstance->labelWithInstance()->label()->label()->c_str());
  EXPECT_FLOAT_EQ(readInstance->labelWithInstance()->label()->confidence(),
                  writeInstance->labelWithInstance()->label()->confidence());
  EXPECT_STREQ(readInstance->labelWithInstance()->instanceUuid()->c_str(),
               writeInstance->labelWithInstance()->instanceUuid()->c_str());

  testEqualPoints(readInstance->bounding_box()->center_point(), writeInstance->bounding_box()->center_point());
  testEqualPoints(readInstance->bounding_box()->spatial_extent(), writeInstance->bounding_box()->spatial_extent());
  EXPECT_FLOAT_EQ(readInstance->bounding_box()->rotation(), writeInstance->bounding_box()->rotation());
}

void testBB2DWithInstanceCategories(const seerep::fb::BoundingBox2DLabeledWithCategory* readInstance,
                                    const seerep::fb::BoundingBox2DLabeledWithCategory* writeInstance)
{
  if (readInstance == nullptr || writeInstance == nullptr)
  {
    FAIL() << "Error: Can't compare a LabelWithInstance to nullptr";
  }
  EXPECT_STREQ(readInstance->category()->c_str(), writeInstance->category()->c_str());
  for (size_t i = 0; i < readInstance->boundingBox2dLabeled()->size(); i++)
  {
    testBB2DWithInstance(readInstance->boundingBox2dLabeled()->Get(i), writeInstance->boundingBox2dLabeled()->Get(i));
  }
}

TEST_F(fbWriteLoadTest, testBoundingBox2DLabeled)
{
  ASSERT_EQ(readImage->labels_bb()->size(), writeImage->labels_bb()->size());
  for (size_t i = 0; i < readImage->labels_bb()->size(); i++)
  {
    testBB2DWithInstanceCategories(readImage->labels_bb()->Get(i), writeImage->labels_bb()->Get(i));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace tests
}  // namespace seerep_hdf5_fb
