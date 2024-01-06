#ifndef DEFS_H
#define DEFS_H

#define PUPIL_TOP_AT_CENTER 3
#define MATRIX_TOP 0
#define MATRIX_BOTTOM 7

#define LEFT_EYE_X  0
#define RIGHT_EYE_X 24

enum PupilPlacement_t : int { eCenter = 0, eNearLeft = 1, eLeft = 2, eFarLeft = 3, eNearRight = 4, eRight = 5, eFarRight = 6 };

enum LidHeight_t : int {
  eOpen = MATRIX_TOP,
  eHigh = (MATRIX_TOP + PUPIL_TOP_AT_CENTER) / 2,
  eMiddle = PUPIL_TOP_AT_CENTER,
  eLow = (PUPIL_TOP_AT_CENTER + MATRIX_BOTTOM) / 2,
  eClosed = MATRIX_BOTTOM
};

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

#endif // DEFS_H
