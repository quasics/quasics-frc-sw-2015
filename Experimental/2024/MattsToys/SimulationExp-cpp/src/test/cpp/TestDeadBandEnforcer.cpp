#include <gtest/gtest.h>

#include "utils/DeadBandEnforcer.h"

TEST(DeadBandEnforcerTests, SimpleConstructor) {
  DeadBandEnforcer deadband(.5);  // Should work out to (-0.5,0.5)

  EXPECT_EQ(deadband(1.0), 1.0);
  EXPECT_EQ(deadband(.51), .51);
  EXPECT_EQ(deadband(.5), .5);
  EXPECT_EQ(deadband(.49999), 0.0);
  EXPECT_EQ(deadband(0.0), 0.0);
  EXPECT_EQ(deadband(-.49999), 0.0);
  EXPECT_EQ(deadband(-.5), -.5);
  EXPECT_EQ(deadband(-.51), -.51);
  EXPECT_EQ(deadband(-1.0), -1.0);
}

TEST(DeadBandEnforcerTests, SimpleConstructorStandardizesBounds) {
  DeadBandEnforcer deadband(-.5);  // Should work out to (-0.5,0.5)

  EXPECT_EQ(deadband(1.0), 1.0);
  EXPECT_EQ(deadband(.51), .51);
  EXPECT_EQ(deadband(.5), .5);
  EXPECT_EQ(deadband(.49999), 0.0);
  EXPECT_EQ(deadband(0.0), 0.0);
  EXPECT_EQ(deadband(-.49999), 0.0);
  EXPECT_EQ(deadband(-.5), -.5);
  EXPECT_EQ(deadband(-.51), -.51);
  EXPECT_EQ(deadband(-1.0), -1.0);
}

TEST(DeadBandEnforcerTests, AssymetricBounding) {
  DeadBandEnforcer deadband(-.25, .75);  // Should work out to (-0.5,0.5)

  EXPECT_EQ(deadband(1.0), 1.0);
  EXPECT_EQ(deadband(.751), .751);
  EXPECT_EQ(deadband(.75), .75);
  EXPECT_EQ(deadband(.749999), 0.0);
  EXPECT_EQ(deadband(0.0), 0.0);
  EXPECT_EQ(deadband(-.249999), 0.0);
  EXPECT_EQ(deadband(-.25), -.25);
  EXPECT_EQ(deadband(-.251), -.251);
  EXPECT_EQ(deadband(-1.0), -1.0);
}

TEST(DeadBandEnforcerTests, AssymetricBoundingStandardizesBounds) {
  DeadBandEnforcer deadband(.75, -.25);  // Should work out to (-0.25,0.75)

  EXPECT_EQ(deadband(1.0), 1.0);
  EXPECT_EQ(deadband(.751), .751);
  EXPECT_EQ(deadband(.75), .75);
  EXPECT_EQ(deadband(.749999), 0.0);
  EXPECT_EQ(deadband(0.0), 0.0);
  EXPECT_EQ(deadband(-.249999), 0.0);
  EXPECT_EQ(deadband(-.25), -.25);
  EXPECT_EQ(deadband(-.251), -.251);
  EXPECT_EQ(deadband(-1.0), -1.0);
}

TEST(DeadBandEnforcerTests, BoundingWithCustomDeadValue) {
  DeadBandEnforcer deadband(-.25, .75, 3);  // Should work out to (-0.5,0.5)

  EXPECT_EQ(deadband(1.0), 1.0);
  EXPECT_EQ(deadband(.751), .751);
  EXPECT_EQ(deadband(.75), .75);
  EXPECT_EQ(deadband(.749999), 3);
  EXPECT_EQ(deadband(0.0), 3);
  EXPECT_EQ(deadband(-.249999), 3);
  EXPECT_EQ(deadband(-.25), -.25);
  EXPECT_EQ(deadband(-.251), -.251);
  EXPECT_EQ(deadband(-1.0), -1.0);
}
