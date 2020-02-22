/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/CameraSpinner.h"

#include <cmath>

#include "Constants.h"

const double CameraSpinner::kPositionTolerance = 0.05;
const double CameraSpinner::kFrontValue = 1.0;
const double CameraSpinner::kRearValue = -1.0;

CameraSpinner::CameraSpinner()
    : cameraRotator(PwmAssignments::CameraSpinnerId) {
  SetSubsystem("Camera Spinner");
}

void CameraSpinner::TurnToFront() {
  cameraRotator.Set(kFrontValue);
}

void CameraSpinner::TurnToRear() {
  cameraRotator.Set(kRearValue);
}

CameraSpinner::Position CameraSpinner::GetPosition() {
  static const WithinToleranceChecker<double> closeToFront{kFrontValue,
                                                           kPositionTolerance};

  static const WithinToleranceChecker<double> closeToRear{kRearValue,
                                                          kPositionTolerance};

  const double currentPosition = cameraRotator.Get();
  if (closeToFront(currentPosition)) {
    return Position::Front;
  } else if (closeToRear(currentPosition)) {
    return Position::Rear;
  } else {
    return Position::Inbetween;
  }
}
