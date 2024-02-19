// See discussions of unit testing in:
// * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
// * https://google.github.io/googletest/

#include <hal/HAL.h>

#include "gtest/gtest.h"

int main(int argc, char** argv) {
  HAL_Initialize(500, 0);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
