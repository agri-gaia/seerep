// Author: Hunaid Hameed
// 21 Sep 2022

#include <gtest/gtest.h>

#include "conversions.cpp"

// Things to test
// Header
// Pointfield
// PointCloud2
// Image
// Point
// Quaternion
// Pose
// PoseStamped
// Vector3
// Vector3Stamped
// Transform
// TransformStamped

// ROS Header
std_msgs::Header createHeader()
{
  std_msgs::Header header;
  header.seq = 2;
  header.stamp.sec = 10;
  header.stamp.nsec = 20;
  header.frame_id = "a_frame_id";

  return header;
}

class rosToFbConversionTest : public testing::Test
{
protected:
  static std_msgs::Header original_header;
  static std_msgs::Header converted_header;

  static void SetUpTestSuite()
  {
    /* This function will create all the elements we want to test
     * */

    // Header Test Start
    // ROS header
    original_header = createHeader();

    // Flatbuffer Header
    flatbuffers::grpc::Message<seerep::fb::Header> fb_header;

    // convert from ROS to Flatbuffer
    fb_header = seerep_ros_conversions_fb::toFlat(original_header, "aprojuuid", "amsguuid");

    // convert from Flatbuffer to ROS
    converted_header = seerep_ros_conversions_fb::toROS(*fb_header.GetRoot());
  }
};

std_msgs::Header rosToFbConversionTest::original_header;
std_msgs::Header rosToFbConversionTest::converted_header;

// test header
TEST_F(rosToFbConversionTest, testHeader)
{
  // expect that original and converted-from-fb are equal
  EXPECT_EQ(original_header.stamp.sec, converted_header.stamp.sec);
  // EXPECT_EQ(original_header()->stamp()->nanos(), converted_header()->stamp()->nanos());
  // EXPECT_STREQ(original_header()->frame_id()->c_str(), converted_header()->frame_id()->c_str());
  // EXPECT_STREQ(original_header()->uuid_project()->c_str(), converted_header()->uuid_project()->c_str());
  // EXPECT_STREQ(original_header()->uuid_msgs()->c_str(), converted_header()->uuid_msgs()->c_str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
