#include <gtest/gtest.h>

#include "ucar_controller/odom_angular_scale.h"

TEST(OdomAngularScale, AppliesCcwScaleToPositiveVelocity) {
  EXPECT_NEAR(
      0.986,
      ucar_controller::scaleOdomAngularVelocity(1.0, 0.986, 1.0),
      1e-9);
}

TEST(OdomAngularScale, AppliesCwScaleToNegativeVelocity) {
  EXPECT_NEAR(
      -0.8,
      ucar_controller::scaleOdomAngularVelocity(-1.0, 0.986, 0.8),
      1e-9);
}

TEST(OdomAngularScale, LeavesZeroUnchanged) {
  EXPECT_DOUBLE_EQ(
      0.0,
      ucar_controller::scaleOdomAngularVelocity(0.0, 0.986, 1.0));
}

TEST(OdomAngularScale, IdentityScalesPreserveLegacyBehavior) {
  EXPECT_DOUBLE_EQ(
      0.7,
      ucar_controller::scaleOdomAngularVelocity(0.7, 1.0, 1.0));
  EXPECT_DOUBLE_EQ(
      -0.7,
      ucar_controller::scaleOdomAngularVelocity(-0.7, 1.0, 1.0));
}

TEST(OdomLateralScale, AppliesOneScaleInBothDirections) {
  EXPECT_NEAR(
      1.015,
      ucar_controller::scaleOdomLateralVelocity(1.0, 1.015),
      1e-9);
  EXPECT_NEAR(
      -1.015,
      ucar_controller::scaleOdomLateralVelocity(-1.0, 1.015),
      1e-9);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
