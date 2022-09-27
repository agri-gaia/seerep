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
// Pose - Done
// PoseStamped - Done
// Vector3 -Done
// Vector3Stamped - Done
// Transform - Done
// TransformStamped - Done
// BoundingBox2DLabeled
// BoundingBox2DLabeledStamped

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

// Quaternion
geometry_msgs::Quaternion createQuaternion()
{
  geometry_msgs::Quaternion q;

  q.x = 5;
  q.y = 7;
  q.z = 3;
  q.w = 9;

  return q;
}

// Pose
geometry_msgs::Pose createPose()
{
  geometry_msgs::Pose pose;

  pose.position = createPoint();
  pose.orientation = createQuaternion();

  return pose;
}

// PoseStamped
geometry_msgs::PoseStamped createPoseStamped()
{
  geometry_msgs::PoseStamped pose_stamped;

  pose_stamped.header = createHeader();
  pose_stamped.pose = createPose();

  return pose_stamped;
}

// Vector3
geometry_msgs::Vector3 createVector3()
{
  geometry_msgs::Vector3 v;

  v.x = 8;
  v.y = 5;
  v.z = 3;

  return v;
}

geometry_msgs::Vector3Stamped createVector3Stamped()
{
  geometry_msgs::Vector3Stamped v3_stamped;

  v3_stamped.header = createHeader();
  v3_stamped.vector = createVector3();

  return v3_stamped;
}

geometry_msgs::Transform createTransform()
{
  geometry_msgs::Transform t;

  t.rotation = createQuaternion();
  t.translation = createVector3();

  return t;
}

geometry_msgs::TransformStamped createTransformStamped()
{
  geometry_msgs::TransformStamped ts;

  ts.header = createHeader();
  ts.child_frame_id = "arbitrary_child_frame_id";
  ts.transform = createTransform();

  return ts;
}

// vision_msgs::ObjectHypothesisWithPose createObjectHypothesisWithPose()
// {
//   vision_msgs::ObjectHypothesisWithPose ohwp;

//   ohwp.id = 4;
//   ohwp.score = 3.3;

//   // create pose with covariance
//   ohwp.pose.pose = createPose();
//   ohwp.pose.covariance = 2.2;

//   return ohwp;
// }

// vision_msgs::Detection2D createDetection2D()
// {
//   vision_msgs::Detection2D d2d;

//   d2d.header = createHeader();

//   // create results
//   std::vector<vision_msgs::ObjectHypothesisWithPose> ohwp;
//   ohwp.push_back(createObjectHypothesisWithPose());
//   ohwp.push_back(createObjectHypothesisWithPose());
//   d2d.results = ohwp;

//   // create BoundingBox
//   // d2d.bbox = createBoundingBox();

//   d2d.source_img = createImage();

//   return d2d;
// }

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

  static geometry_msgs::Quaternion original_q;
  static geometry_msgs::Quaternion converted_q;

  static geometry_msgs::Pose original_pose;
  static geometry_msgs::Pose converted_pose;

  static geometry_msgs::PoseStamped original_pose_stamped;
  static geometry_msgs::PoseStamped converted_pose_stamped;

  static geometry_msgs::Vector3 original_v;
  static geometry_msgs::Vector3 converted_v;

  static geometry_msgs::Vector3Stamped original_v3_stamped;
  static geometry_msgs::Vector3Stamped converted_v3_stamped;

  static geometry_msgs::Transform original_t;
  static geometry_msgs::Transform converted_t;

  static geometry_msgs::TransformStamped original_t_stamped;
  static geometry_msgs::TransformStamped converted_t_stamped;

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

    // Quaternion Start
    geometry_msgs::Quaternion original_q = createQuaternion();

    flatbuffers::grpc::Message<seerep::fb::Quaternion> fb_quaternion;
    fb_quaternion = seerep_ros_conversions_fb::toFlat(original_q);

    geometry_msgs::Quaternion converted_q = seerep_ros_conversions_fb::toROS(*fb_quaternion.GetRoot());
    // Quaternion End

    // Pose Start
    geometry_msgs::Pose original_pose = createPose();

    flatbuffers::grpc::Message<seerep::fb::Pose> fb_pose;
    fb_pose = seerep_ros_conversions_fb::toFlat(original_pose);

    geometry_msgs::Pose converted_pose = seerep_ros_conversions_fb::toROS(*fb_pose.GetRoot());
    // Pose End

    // PoseStamped Start
    geometry_msgs::PoseStamped original_pose_stamped = createPoseStamped();

    flatbuffers::grpc::Message<seerep::fb::PoseStamped> fb_pose_stamped;
    fb_pose_stamped = seerep_ros_conversions_fb::toFlat(original_pose_stamped, p_uuid);

    geometry_msgs::PoseStamped converted_pose_stamped = seerep_ros_conversions_fb::toROS(*fb_pose_stamped.GetRoot());
    // PoseStamped End

    // Vector3 Start
    geometry_msgs::Vector3 original_v = createVector3();

    flatbuffers::grpc::Message<seerep::fb::Vector3> fb_vector3;
    fb_vector3 = seerep_ros_conversions_fb::toFlat(original_v);

    geometry_msgs::Vector3 converted_v = seerep_ros_conversions_fb::toROS(*fb_vector3.GetRoot());
    // Vector3 End

    // Vector3Stamped Start
    geometry_msgs::Vector3Stamped original_v3_stamped = createVector3Stamped();

    flatbuffers::grpc::Message<seerep::fb::Vector3Stamped> fb_vector3_stamped;
    fb_vector3_stamped = seerep_ros_conversions_fb::toFlat(original_v3_stamped, p_uuid);

    geometry_msgs::Vector3Stamped converted_v3_stamped =
        seerep_ros_conversions_fb::toROS(*fb_vector3_stamped.GetRoot());
    // Vector3Stamped End

    // Transform Start
    geometry_msgs::Transform original_t = createTransform();

    flatbuffers::grpc::Message<seerep::fb::Transform> fb_transform;
    fb_transform = seerep_ros_conversions_fb::toFlat(original_t);

    geometry_msgs::Transform converted_t = seerep_ros_conversions_fb::toROS(*fb_transform.GetRoot());
    // Transform End

    // TransformStamped Start
    geometry_msgs::TransformStamped original_t_stamped = createTransformStamped();

    flatbuffers::grpc::Message<seerep::fb::TransformStamped> fb_transform_stamped;
    fb_transform_stamped = seerep_ros_conversions_fb::toFlat(original_t_stamped, p_uuid);

    geometry_msgs::TransformStamped converted_t_stamped =
        seerep_ros_conversions_fb::toROS(*fb_transform_stamped.GetRoot());
    // Transform End
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

// Point
geometry_msgs::Point rosToFbConversionTest::original_p;
geometry_msgs::Point rosToFbConversionTest::converted_p;

// Quaternion
geometry_msgs::Quaternion rosToFbConversionTest::original_q;
geometry_msgs::Quaternion rosToFbConversionTest::converted_q;

// Pose
geometry_msgs::Pose rosToFbConversionTest::original_pose;
geometry_msgs::Pose rosToFbConversionTest::converted_pose;

// PoseStamped
geometry_msgs::PoseStamped rosToFbConversionTest::original_pose_stamped;
geometry_msgs::PoseStamped rosToFbConversionTest::converted_pose_stamped;

// Vector3
geometry_msgs::Vector3 rosToFbConversionTest::original_v;
geometry_msgs::Vector3 rosToFbConversionTest::converted_v;

// Vector3Stamped
geometry_msgs::Vector3Stamped rosToFbConversionTest::original_v3_stamped;
geometry_msgs::Vector3Stamped rosToFbConversionTest::converted_v3_stamped;

// Transform
geometry_msgs::Transform rosToFbConversionTest::original_t;
geometry_msgs::Transform rosToFbConversionTest::converted_t;

// TransformStamped
geometry_msgs::TransformStamped rosToFbConversionTest::original_t_stamped;
geometry_msgs::TransformStamped rosToFbConversionTest::converted_t_stamped;

void testHeader(std_msgs::Header original_header, std_msgs::Header converted_header)
{
  EXPECT_EQ(original_header.seq, converted_header.seq);
  EXPECT_EQ(original_header.stamp.sec, converted_header.stamp.sec);
  EXPECT_EQ(original_header.stamp.nsec, converted_header.stamp.nsec);
  EXPECT_STREQ(original_header.frame_id.c_str(), converted_header.frame_id.c_str());
  // EXPECT_EQ(original_header.uuid_project.c_str, converted_header.uuid_project.c_str);
  // EXPECT_EQ(original_header.uuid_msgs.c_str, converted_header.uuid_msgs.c_str);
}

// test header
TEST_F(rosToFbConversionTest, testHeader)
{
  // expect that original and converted-from-fb are equal
  testHeader(original_header, converted_header);
}

TEST_F(rosToFbConversionTest, testPointField)
{
  EXPECT_STREQ(original_pf.name.c_str(), converted_pf.name.c_str());
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
  EXPECT_EQ(original_pc2.data, converted_pc2.data);  // test in a loop separately

  EXPECT_EQ(original_pc2.is_dense, converted_pc2.is_dense);
}

TEST_F(rosToFbConversionTest, testPointCloud2Data)
{
  for (size_t i = 0; i < original_pc2.data.size(); i++)
  {
    EXPECT_EQ(original_pc2.data[i], converted_pc2.data[i]);
  }
}

TEST_F(rosToFbConversionTest, testImage)
{
  testHeader(original_img.header, converted_img.header);
  EXPECT_EQ(original_img.height, converted_img.height);
  EXPECT_EQ(original_img.width, converted_img.width);
  EXPECT_STREQ(original_img.encoding.c_str(), converted_img.encoding.c_str());
  EXPECT_EQ(original_img.is_bigendian, converted_img.is_bigendian);
  EXPECT_EQ(original_img.step, converted_img.step);
}

TEST_F(rosToFbConversionTest, testImageData)
{
  for (size_t i = 0; i < original_img.data.size(); i++)
  {
    EXPECT_EQ(original_img.data[i], converted_img.data[i]);
  }
}

TEST_F(rosToFbConversionTest, testImgData)
{
  for (size_t i = 0; i < original_img.data.size(); i++)
  {
    EXPECT_EQ(original_img.data[i], converted_img.data[i]);
  }
}

void testPoint(geometry_msgs::Point original_p, geometry_msgs::Point converted_p)
{
  EXPECT_EQ(original_p.x, converted_p.x);
  EXPECT_EQ(original_p.y, converted_p.y);
  EXPECT_EQ(original_p.z, converted_p.z);
}

TEST_F(rosToFbConversionTest, testPoint)
{
  testPoint(original_p, converted_p);
}

void testQuaternion(geometry_msgs::Quaternion original_q, geometry_msgs::Quaternion converted_q)
{
  EXPECT_EQ(original_q.x, converted_q.x);
  EXPECT_EQ(original_q.y, converted_q.y);
  EXPECT_EQ(original_q.z, converted_q.z);
  EXPECT_EQ(original_q.w, converted_q.w);
}

TEST_F(rosToFbConversionTest, testQuaternion)
{
  testQuaternion(original_q, converted_q);
}

TEST_F(rosToFbConversionTest, testPose)
{
  testPoint(original_pose.position, converted_pose.position);
  testQuaternion(original_pose.orientation, converted_pose.orientation);
}

TEST_F(rosToFbConversionTest, testPoseStamped)
{
  // test the header
  testHeader(original_pose_stamped.header, converted_pose_stamped.header);

  // test the pose
  testPoint(original_pose_stamped.pose.position, converted_pose_stamped.pose.position);
  testQuaternion(original_pose_stamped.pose.orientation, converted_pose_stamped.pose.orientation);
}

void testVector3(geometry_msgs::Vector3 original_v, geometry_msgs::Vector3 converted_v)
{
  EXPECT_EQ(original_v.x, converted_v.x);
  EXPECT_EQ(original_v.y, converted_v.y);
  EXPECT_EQ(original_v.z, converted_v.z);
}

TEST_F(rosToFbConversionTest, testVector3)
{
  testVector3(original_v, converted_v);
}

TEST_F(rosToFbConversionTest, testVector3Stamped)
{
  testHeader(original_v3_stamped.header, original_v3_stamped.header);

  testVector3(original_v3_stamped.vector, converted_v3_stamped.vector);
}

TEST_F(rosToFbConversionTest, testTransform)
{
  testVector3(original_t.translation, converted_t.translation);
  testQuaternion(original_t.rotation, converted_t.rotation);
}

TEST_F(rosToFbConversionTest, testTransformStamped)
{
  testHeader(original_t_stamped.header, converted_t_stamped.header);

  EXPECT_STREQ(original_t_stamped.child_frame_id.c_str(), converted_t_stamped.child_frame_id.c_str());

  testVector3(original_t.translation, converted_t.translation);
  testQuaternion(original_t.rotation, converted_t.rotation);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
