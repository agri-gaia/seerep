// Author: Hunaid Hameed
// 21 Sep 2022

#include <gtest/gtest.h>

#include "seerep_ros_conversions_fb/conversions.h"

// Things to test
// Header - Done
// Pointfield - Done
// PointCloud2 - Done
// Image - Done
// Point - Done
// Quaternion - Done
// Vector3 -Done
// Transform - Done
// TransformStamped - Done
// BoundingBox2DLabeled
// BoundingBox2DLabeledStamped

/**
 * @brief creates a ros std_msgs header with arbitrary values
 * @return std_msgs::Header
 * */
std_msgs::Header createHeader()
{
  std_msgs::Header header;
  header.seq = 2;
  header.stamp.sec = 10;
  header.stamp.nsec = 20;
  header.frame_id = "a_frame_id";

  return header;
}

/**
 * @brief creates a ros sensor_msgs pointfield with arbitrary values
 * @return sensor_msgs::PointField
 * */
sensor_msgs::PointField createPointField()
{
  sensor_msgs::PointField pf;

  pf.name = "pf_name";
  pf.offset = 3;
  pf.datatype = 7;
  pf.count = 99;

  return pf;
}

/**
 * @brief creates a ros sensor_msgs pointcloud with arbitrary values
 * @return sensor_msgs::PointCloud2
 * */
sensor_msgs::PointCloud2 createPointCloud()
{
  sensor_msgs::PointCloud2 pc2;

  pc2.header = createHeader();

  pc2.height = 5;
  pc2.width = 6;

  // create two pointfield objects in a vector and assign it to the point count fields attribute
  std::vector<sensor_msgs::PointField> pf = { createPointField(),
                                              createPointField() };
  pc2.fields = pf;

  pc2.is_bigendian = true;
  pc2.point_step = 15;
  pc2.row_step = 20;

  std::vector<uint8_t> d;
  for (unsigned int h = 0; h < pc2.height; h++)
  {
    for (unsigned int w = 0; w < pc2.width; w++)
    {
      // arbitrarily populate the d vector with the sum of h and w
      d.push_back(h + w);
    }
  }
  pc2.data = d;

  pc2.is_dense = true;

  return pc2;
}

/**
 * @brief creates a ros sensor_msgs image with arbitrary values
 * @return sensor_msgs::Image
 * */
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
  for (unsigned int h = 0; h < img.height; h++)
  {
    for (unsigned int w = 0; w < img.width; w++)
    {
      // arbitrariry populate the d vector with the sum of h and w
      d.push_back(h + w);
    }
  }
  img.data = d;

  return img;
}

/**
 * @brief creates a ros geometry_msgs point with arbitrary values
 * @return geometry_msgs::Point
 * */
geometry_msgs::Point createPoint()
{
  geometry_msgs::Point p;

  p.x = 4;
  p.y = 5;
  p.z = 6;

  return p;
}

/**
 * @brief creates a ros geometry_msgs quaternion with arbitrary values
 * @return geometry_msgs::Quaternion
 * */
geometry_msgs::Quaternion createQuaternion()
{
  geometry_msgs::Quaternion q;

  q.x = 5;
  q.y = 7;
  q.z = 3;
  q.w = 9;

  return q;
}

/**
 * @brief creates a ros geometry_msgs vector3 with arbitrary values
 * @return geometry_msgs::Vector3
 * */
geometry_msgs::Vector3 createVector3()
{
  geometry_msgs::Vector3 v;

  v.x = 8;
  v.y = 5;
  v.z = 3;

  return v;
}

/**
 * @brief creates a ros geometry_msgs transform with arbitrary values
 * @return geometry_msgs::Vector3Stamped
 * */
geometry_msgs::Transform createTransform()
{
  geometry_msgs::Transform t;

  t.rotation = createQuaternion();
  t.translation = createVector3();

  return t;
}

/**
 * @brief creates a ros geometry_msgs transformstamped with arbitrary values
 * @return geometry_msgs::TransformStamped
 * */
geometry_msgs::TransformStamped createTransformStamped()
{
  geometry_msgs::TransformStamped ts;

  ts.header = createHeader();
  ts.child_frame_id = "arbitrary_child_frame_id";
  ts.transform = createTransform();

  return ts;
}

/**
 * @brief This class holds all the attributes which will be tested in the tests.
 * */
class rosToFbConversionTest : public testing::Test
{
public:
  std_msgs::Header original_header;
  std_msgs::Header converted_header;

  sensor_msgs::PointField original_pf;
  sensor_msgs::PointField converted_pf;

  sensor_msgs::PointCloud2 original_pc2;
  sensor_msgs::PointCloud2 converted_pc2;

  sensor_msgs::Image original_img;
  sensor_msgs::Image converted_img;

  geometry_msgs::Point original_p;
  geometry_msgs::Point converted_p;

  geometry_msgs::Quaternion original_q;
  geometry_msgs::Quaternion converted_q;

  geometry_msgs::Vector3 original_v;
  geometry_msgs::Vector3 converted_v;

  geometry_msgs::Transform original_t;
  geometry_msgs::Transform converted_t;

  geometry_msgs::TransformStamped original_t_stamped;
  geometry_msgs::TransformStamped converted_t_stamped;

  std::string p_uuid = "aprojuuid";
  std::string m_uuid = "amsguuid";

  void createHeaderObjects()
  {
    // Header Test Start
    // ROS header
    original_header = createHeader();

    // Flatbuffer Header
    flatbuffers::grpc::Message<seerep::fb::Header> fb_header;

    // convert from ROS to Flatbuffer
    fb_header =
        seerep_ros_conversions_fb::toFlat(original_header, p_uuid, m_uuid);

    // convert from Flatbuffer to ROS
    converted_header = seerep_ros_conversions_fb::toROS(*fb_header.GetRoot());
    // Header Test End
  }

  void createPointFieldObjects()
  {
    // PointField Test Start
    original_pf = createPointField();

    flatbuffers::grpc::Message<seerep::fb::PointField> fb_pointfield;
    fb_pointfield = seerep_ros_conversions_fb::toFlat(original_pf);

    converted_pf = seerep_ros_conversions_fb::toROS(*fb_pointfield.GetRoot());
    // PointField Test End
  }

  void createPointCloud2Objects()
  {
    // PointCloud2 Test Start
    original_pc2 = createPointCloud();

    flatbuffers::grpc::Message<seerep::fb::PointCloud2> fb_pointcloud2;
    fb_pointcloud2 =
        seerep_ros_conversions_fb::toFlat(original_pc2, p_uuid, m_uuid);

    converted_pc2 = seerep_ros_conversions_fb::toROS(*fb_pointcloud2.GetRoot());
    // PointCloud2 Test End
  }

  void createImageObjects()
  {
    // Image Test Start
    original_img = createImage();

    flatbuffers::grpc::Message<seerep::fb::Image> fb_image;
    fb_image = seerep_ros_conversions_fb::toFlat(original_img, p_uuid, m_uuid);

    converted_img = seerep_ros_conversions_fb::toROS(*fb_image.GetRoot());
    // Image Test End
  }

  void createPointObjects()
  {
    // Point Start
    original_p = createPoint();

    flatbuffers::grpc::Message<seerep::fb::Point> fb_point;
    fb_point = seerep_ros_conversions_fb::toFlat(original_p);

    converted_p = seerep_ros_conversions_fb::toROS(*fb_point.GetRoot());
    // Point End
  }

  void createQuaternionObjects()
  {
    // Quaternion Start
    original_q = createQuaternion();

    flatbuffers::grpc::Message<seerep::fb::Quaternion> fb_quaternion;
    fb_quaternion = seerep_ros_conversions_fb::toFlat(original_q);

    converted_q = seerep_ros_conversions_fb::toROS(*fb_quaternion.GetRoot());
    // Quaternion End
  }

  void createVector3Objects()
  {
    // Vector3 Start
    original_v = createVector3();

    flatbuffers::grpc::Message<seerep::fb::Vector3> fb_vector3;
    fb_vector3 = seerep_ros_conversions_fb::toFlat(original_v);

    converted_v = seerep_ros_conversions_fb::toROS(*fb_vector3.GetRoot());
    // Vector3 End
  }

  void createTransformObjects()
  {
    // Transform Start
    original_t = createTransform();

    flatbuffers::grpc::Message<seerep::fb::Transform> fb_transform;
    fb_transform = seerep_ros_conversions_fb::toFlat(original_t);

    converted_t = seerep_ros_conversions_fb::toROS(*fb_transform.GetRoot());
    // Transform End
  }

  void createTransformStampedObjects()
  {
    // TransformStamped Start
    original_t_stamped = createTransformStamped();

    flatbuffers::grpc::Message<seerep::fb::TransformStamped> fb_transform_stamped;
    fb_transform_stamped =
        seerep_ros_conversions_fb::toFlat(original_t_stamped, p_uuid, false);

    converted_t_stamped =
        seerep_ros_conversions_fb::toROS(*fb_transform_stamped.GetRoot());
    // Transform End
  }

  /**
   * @brief This function instantiates all the attributes and fills them with
   * arbitrary values.
   * */
  rosToFbConversionTest()
  {
    /* This function will create all the elements we want to test
     * */

    createHeaderObjects();

    createPointFieldObjects();

    createPointCloud2Objects();

    createImageObjects();

    createPointObjects();

    createQuaternionObjects();

    createVector3Objects();

    createTransformObjects();

    createTransformStampedObjects();
  }
};

/**
 * @brief Given two ROS std_msgs::Header instances, this function tests their
 * sub attributes of equality.
 * */
void testHeader(std_msgs::Header original_header,
                std_msgs::Header converted_header)
{
  EXPECT_EQ(original_header.seq, converted_header.seq);
  EXPECT_EQ(original_header.stamp.sec, converted_header.stamp.sec);
  EXPECT_EQ(original_header.stamp.nsec, converted_header.stamp.nsec);
  EXPECT_STREQ(original_header.frame_id.c_str(),
               converted_header.frame_id.c_str());
}

// test header
/**
 * @brief Test Header for Equality
 * */
TEST_F(rosToFbConversionTest, testHeader)
{
  // expect that original and converted-from-fb are equal
  testHeader(original_header, converted_header);
}

/**
 * @brief Test PointField for Equality
 * */
TEST_F(rosToFbConversionTest, testPointField)
{
  EXPECT_STREQ(original_pf.name.c_str(), converted_pf.name.c_str());
  EXPECT_EQ(original_pf.offset, converted_pf.offset);
  EXPECT_EQ(original_pf.datatype, converted_pf.datatype);
  EXPECT_EQ(original_pf.count, converted_pf.count);
}

/**
 * @brief Test PointCloud2 for Equality
 * */
TEST_F(rosToFbConversionTest, testPointCloud2)
{
  EXPECT_EQ(original_pc2.header, converted_pc2.header);

  EXPECT_EQ(original_pc2.height, converted_pc2.height);
  EXPECT_EQ(original_pc2.width, converted_pc2.width);

  EXPECT_EQ(original_pc2.fields, converted_pc2.fields);

  EXPECT_EQ(original_pc2.is_bigendian, converted_pc2.is_bigendian);
  EXPECT_EQ(original_pc2.point_step, converted_pc2.point_step);
  EXPECT_EQ(original_pc2.row_step, converted_pc2.row_step);
  EXPECT_EQ(original_pc2.data,
            converted_pc2.data);  // test in a loop separately

  EXPECT_EQ(original_pc2.is_dense, converted_pc2.is_dense);
}

/**
 * @brief Test PointCloud2Data for Equality
 * */
TEST_F(rosToFbConversionTest, testPointCloud2Data)
{
  for (size_t i = 0; i < original_pc2.data.size(); i++)
  {
    EXPECT_EQ(original_pc2.data[i], converted_pc2.data[i]);
  }
}

/**
 * @brief Test Image for Equality
 * */
TEST_F(rosToFbConversionTest, testImage)
{
  testHeader(original_img.header, converted_img.header);
  EXPECT_EQ(original_img.height, converted_img.height);
  EXPECT_EQ(original_img.width, converted_img.width);
  EXPECT_STREQ(original_img.encoding.c_str(), converted_img.encoding.c_str());
  EXPECT_EQ(original_img.is_bigendian, converted_img.is_bigendian);
  EXPECT_EQ(original_img.step, converted_img.step);
}

/**
 * @brief Test Image for Equality
 * */
TEST_F(rosToFbConversionTest, testImageData)
{
  for (size_t i = 0; i < original_img.data.size(); i++)
  {
    EXPECT_EQ(original_img.data[i], converted_img.data[i]);
  }
}

/**
 * @brief Test Image Data for Equality
 * */
TEST_F(rosToFbConversionTest, testImgData)
{
  for (size_t i = 0; i < original_img.data.size(); i++)
  {
    EXPECT_EQ(original_img.data[i], converted_img.data[i]);
  }
}

/**
 * @brief Given two ROS geometry_msgs::Point instances, this function tests
 * their sub attributes of equality.
 * */
void testPoint(geometry_msgs::Point original_p, geometry_msgs::Point converted_p)
{
  EXPECT_EQ(original_p.x, converted_p.x);
  EXPECT_EQ(original_p.y, converted_p.y);
  EXPECT_EQ(original_p.z, converted_p.z);
}

/**
 * @brief Test Point for Equality
 * */
TEST_F(rosToFbConversionTest, testPoint)
{
  testPoint(original_p, converted_p);
}

/**
 * @brief Given two ROS geometry_msgs::Quaternion instances, this function tests
 * their sub attributes of equality.
 * */
void testQuaternion(geometry_msgs::Quaternion original_q,
                    geometry_msgs::Quaternion converted_q)
{
  EXPECT_EQ(original_q.x, converted_q.x);
  EXPECT_EQ(original_q.y, converted_q.y);
  EXPECT_EQ(original_q.z, converted_q.z);
  EXPECT_EQ(original_q.w, converted_q.w);
}

/**
 * @brief Test Point for Equality
 * */
TEST_F(rosToFbConversionTest, testQuaternion)
{
  testQuaternion(original_q, converted_q);
}

/**
 * @brief Given two ROS geometry_msgs::Vector3 instances, this function tests
 * their sub attributes of equality.
 * */
void testVector3(geometry_msgs::Vector3 original_v,
                 geometry_msgs::Vector3 converted_v)
{
  EXPECT_EQ(original_v.x, converted_v.x);
  EXPECT_EQ(original_v.y, converted_v.y);
  EXPECT_EQ(original_v.z, converted_v.z);
}

/**
 * @brief Test Vector3 for Equality
 * */
TEST_F(rosToFbConversionTest, testVector3)
{
  testVector3(original_v, converted_v);
}

/**
 * @brief Test Transform for Equality
 * */
TEST_F(rosToFbConversionTest, testTransform)
{
  testVector3(original_t.translation, converted_t.translation);
  testQuaternion(original_t.rotation, converted_t.rotation);
}

/**
 * @brief Test TransformStamped for Equality
 * */
TEST_F(rosToFbConversionTest, testTransformStamped)
{
  testHeader(original_t_stamped.header, converted_t_stamped.header);

  EXPECT_STREQ(original_t_stamped.child_frame_id.c_str(),
               converted_t_stamped.child_frame_id.c_str());

  testVector3(original_t.translation, converted_t.translation);
  testQuaternion(original_t.rotation, converted_t.rotation);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
