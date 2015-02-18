#include "DriveTrainSmoothing.h"
#include <cstdio>

using std::printf;

int main() {
  printf("Using linear scaling:\n");
  for( float input = -1.0f; input <= +1.0; input += joystickStep / 2 ) {
    float left, right;
    TrimJoystickValuesToPowerWithLinearScaling(input, input, left, right);
    printf("For input %f, use multipliers left: %f, right: %f\n",
           input, left, right);
  }
  
  printf("Without linear scaling:\n");
  for( float input = -1.0f; input <= +1.0; input += joystickStep / 2 ) {
    float left, right;
    TrimJoystickValuesToPowerWithoutLinearScaling(input, input, left, right);
    printf("For input %f, use multipliers left: %f, right: %f\n",
           input, left, right);
  }
  return 0;
}