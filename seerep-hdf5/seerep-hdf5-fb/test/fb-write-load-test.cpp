#include <gtest/gtest.h>

TEST(WriteTests, firstTest)
{
  ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
