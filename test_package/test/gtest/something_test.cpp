// Bring in my package's API, which is what I'm testing
#include <something.h>
// Bring in gtest
#include <gtest/gtest.h>

//#include "ros/ros.h"

// Declare a test
TEST(TestSuite, testCase1)
{
  Something c = Something();
  EXPECT_EQ(true, c.this_should_be_tested(1));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
