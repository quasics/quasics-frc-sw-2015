// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"
#include <rev/CANSparkMax.h>

Drivebase::Drivebase() = default;

// This method will be called once per scheduler run
void Drivebase::Periodic() {}
//Todo - Write code for the following 4 commands
void DriveBase::Stop(){
    
};

void SetMotorPower(double LeftPower,double RightPower){

};

double GetLeftEncoders(){

};

double GetRightEncoders(){

};

void ResetEncoders(){

};