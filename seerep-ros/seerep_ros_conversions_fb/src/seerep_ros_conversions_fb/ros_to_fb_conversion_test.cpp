// Author: Hunaid Hameed
// 21 Sep 2022

#include <gtest/gtest.h>

// #include <boost/uuid/uuid.hpp>
// #include <boost/uuid/uuid_generators.hpp>

#include "conversions.cpp"

// Things to test
// Header - Done
// Pointfield - Done
// PointCloud2 - Done
// Image - Done
// Point - Done
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

  // Question for Mark
  // Should we test project_uuid and msg_uuid here?
  // It is a part of the seerep header but not the ROS header, and the toROS function doesn't convert it.

  return header;
}

// Point Field
sensor_msgs::PointField createPointField()
{
  sensor_msgs::PointField pf;

  pf.name = "pf_name";
  pf.offset = 3;
  pf.datatype = 7;
  pf.count = 99;

  return pf;
}

// PointCloud2
sensor_msgs::PointCloud2 createPointCloud()
{
  sensor_msgs::PointCloud2 pc2;

  pc2.header = createHeader();

  pc2.height = 5;
  pc2.width = 6;

  // create two pointfield objects in a vector and assign it to the point count fields attribute
  std::vector<sensor_msgs::PointField> pf = { createPointField(), createPointField() };
  pc2.fields = pf;

  pc2.is_bigendian = true;
  pc2.point_step = 15;
  pc2.row_step = 20;

  std::vector<uint8_t> d;
  for (int h = 0; h < pc2.height; h++)
  {
    for (int w = 0; w < pc2.width; w++)
    {
      // arbitrariry populate the d vector with the sum of h and w
      d.push_back(h + w);
    }
  }
  pc2.data = d;

  pc2.is_dense = true;

  return pc2;
}

// Image
sensor_msgs::Image createImage()
{
  sensor_msgs::Image img;

  img.header = createHeader();

  img.height = 5;
  img.width = 6;

  img.encoding = "arbitrary_encoding_type";

  img.is_bigendian = true;
  img.step = 44;

  std::vector<uint8_t> d;
  for (int h = 0; h < img.height; h++)
  {
    for (int w = 0; w < img.width; w++)
    {
      // arbitrariry populate the d vector with the sum of h and w
      d.push_back(h + w);
    }
  }
  img.data = d;

  return img;
}

// Point
geometry_msgs::Point createPoint()
{
  geometry_msgs::Point p;

  p.x = 4;
  p.y = 5;
  p.z = 6;

  return p;
}

class rosToFbConversionTest : public testing::Test
{
protected:
  static std_msgs::Header original_header;
  static std_msgs::Header converted_header;

  static sensor_msgs::PointField original_pf;
  static sensor_msgs::PointField converted_pf;

  static sensor_msgs::PointCloud2 original_pc2;
  static sensor_msgs::PointCloud2 converted_pc2;

  static sensor_msgs::Image original_img;
  static sensor_msgs::Image converted_img;

  static geometry_msgs::Point original_p;
  static geometry_msgs::Point converted_p;

  static void SetUpTestSuite()
  {
    /* This function will create all the elements we want to test
     * */

    std::string p_uuid = "aprojuuid";
    std::string m_uuid = "amsguuid";

    // Header Test Start
    // ROS header
    original_header = createHeader();

    // Flatbuffer Header
    flatbuffers::grpc::Message<seerep::fb::Header> fb_header;

    // convert from ROS to Flatbuffer
    fb_header = seerep_ros_conversions_fb::toFlat(original_header, p_uuid, m_uuid);

    // convert from Flatbuffer to ROS
    converted_header = seerep_ros_conversions_fb::toROS(*fb_header.GetRoot());
    // Header Test End

    // PointField Test Start
    sensor_msgs::PointField original_pf = createPointField();

    flatbuffers::grpc::Message<seerep::fb::PointField> fb_pointfield;
    fb_pointfield = seerep_ros_conversions_fb::toFlat(original_pf);

    sensor_msgs::PointField converted_pf = seerep_ros_conversions_fb::toROS(*fb_pointfield.GetRoot());
    // PointField Test End

    // PointCloud2 Test Start
    original_pc2 = createPointCloud();

    flatbuffers::grpc::Message<seerep::fb::PointCloud2> fb_pointcloud2;
    fb_pointcloud2 = seerep_ros_conversions_fb::toFlat(original_pc2, p_uuid, m_uuid);

    converted_pc2 = seerep_ros_conversions_fb::toROS(*fb_pointcloud2.GetRoot());
    // PointCloud2 Test End

    // Image Test Start
    sensor_msgs::Image original_img = createImage();

    flatbuffers::grpc::Message<seerep::fb::Image> fb_image;
    fb_image = seerep_ros_conversions_fb::toFlat(original_img, p_uuid, m_uuid);

    sensor_msgs::Image converted_img = seerep_ros_conversions_fb::toROS(*fb_image.GetRoot());
    // Image Test End

    // Point Start
    geometry_msgs::Point original_p = createPoint();

    flatbuffers::grpc::Message<seerep::fb::Point> fb_point;
    fb_point = seerep_ros_conversions_fb::toFlat(original_p);

    geometry_msgs::Point converted_p = seerep_ros_conversions_fb::toROS(*fb_point.GetRoot());
    // Point End
  }
};

// Objects created below are used by the TEST functions

// Header
std_msgs::Header rosToFbConversionTest::original_header;
std_msgs::Header rosToFbConversionTest::converted_header;

// PointField
sensor_msgs::PointField rosToFbConversionTest::original_pf;
sensor_msgs::PointField rosToFbConversionTest::converted_pf;

// PointCloud2
sensor_msgs::PointCloud2 rosToFbConversionTest::original_pc2;
sensor_msgs::PointCloud2 rosToFbConversionTest::converted_pc2;

// Image
sensor_msgs::Image rosToFbConversionTest::original_img;
sensor_msgs::Image rosToFbConversionTest::converted_img;

geometry_msgs::Point rosToFbConversionTest::original_p;
geometry_msgs::Point rosToFbConversionTest::converted_p;

// test header
TEST_F(rosToFbConversionTest, testHeader)
{
  // expect that original and converted-from-fb are equal
  EXPECT_EQ(original_header.seq, converted_header.seq);
  EXPECT_EQ(original_header.stamp.sec, converted_header.stamp.sec);
  EXPECT_EQ(original_header.stamp.nsec, converted_header.stamp.nsec);
  EXPECT_EQ(original_header.frame_id, converted_header.frame_id);
  // EXPECT_EQ(original_header.uuid_project.c_str, converted_header.uuid_project.c_str);
  // EXPECT_EQ(original_header.uuid_msgs.c_str, converted_header.uuid_msgs.c_str);
}

TEST_F(rosToFbConversionTest, testPointField)
{
  EXPECT_EQ(original_pf.name, converted_pf.name);
  EXPECT_EQ(original_pf.offset, converted_pf.offset);
  EXPECT_EQ(original_pf.datatype, converted_pf.datatype);
  EXPECT_EQ(original_pf.count, converted_pf.count);
}

TEST_F(rosToFbConversionTest, testPointCloud2)
{
  EXPECT_EQ(original_pc2.header, converted_pc2.header);

  EXPECT_EQ(original_pc2.height, converted_pc2.height);
  EXPECT_EQ(original_pc2.width, converted_pc2.width);

  EXPECT_EQ(original_pc2.fields, converted_pc2.fields);

  EXPECT_EQ(original_pc2.is_bigendian, converted_pc2.is_bigendian);
  EXPECT_EQ(original_pc2.point_step, converted_pc2.point_step);
  EXPECT_EQ(original_pc2.row_step, converted_pc2.row_step);
  EXPECT_EQ(original_pc2.data, converted_pc2.data);

  EXPECT_EQ(original_pc2.is_dense, converted_pc2.is_dense);
}

TEST_F(rosToFbConversionTest, testImage)
{
  EXPECT_EQ(original_img.header, converted_img.header);
  EXPECT_EQ(original_img.height, converted_img.height);
  EXPECT_EQ(original_img.width, converted_img.width);
  EXPECT_EQ(original_img.encoding, converted_img.encoding);
  EXPECT_EQ(original_img.is_bigendian, converted_img.is_bigendian);
  EXPECT_EQ(original_img.step, converted_img.step);
  EXPECT_EQ(original_img.data, converted_img.data);
}

TEST_F(rosToFbConversionTest, testPoint)
{
  EXPECT_EQ(original_p.x, converted_p.x);
  EXPECT_EQ(original_p.y, converted_p.y);
  EXPECT_EQ(original_p.z, converted_p.z);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
