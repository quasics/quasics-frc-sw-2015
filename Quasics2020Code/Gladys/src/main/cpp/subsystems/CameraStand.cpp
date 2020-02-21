/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/CameraStand.h"

CameraStand::CameraStand() {

}

// This method will be called once per scheduler run
void CameraStand::Periodic() {}

void CameraStand::TurnCameraMax(){
   double Max = standServo.GetMaxAngle();
   standServo.SetAngle(Max);
}

void CameraStand::TurnCameraMin(){
    double Min = standServo.GetMinAngle();
    standServo.SetAngle(Min);
}
