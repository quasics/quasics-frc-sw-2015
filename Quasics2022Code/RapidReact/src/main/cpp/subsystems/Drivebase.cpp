// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"
#include <rev/CANSparkMax.h>
#include <iostream>
#include <wpi/numbers>

Drivebase::Drivebase(){
 m_rightFront.SetInverted(true);
 m_rightBack.SetInverted(true);



 m_leftSide.reset(new frc::MotorControllerGroup(m_leftFront, m_leftBack));
 m_rightSide.reset(new frc::MotorControllerGroup(m_rightFront, m_rightBack));

 m_drive.reset(new frc::DifferentialDrive(*m_leftSide, *m_rightSide));
}

// This method will be called once per scheduler run

void Drivebase::ConfigureEncoders(){
    const units::meter_t wheelCircumference = WHEEL_DIAMETER * wpi::numbers::pi;
    const units::meter_t gearingConversion = wheelCircumference / DRIVEBASE_GEAR_RATIO;
    const units::meter_t velocityCorrection = gearingConversion / 60;

    m_leftFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
    m_leftBackEncoder.SetPositionConversionFactor(gearingConversion.value());
    m_rightFrontEncoder.SetPositionConversionFactor(gearingConversion.value());
    m_rightBackEncoder.SetPositionConversionFactor(gearingConversion.value());
    
    m_leftFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
    m_leftBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());
    m_rightFrontEncoder.SetVelocityConversionFactor(velocityCorrection.value());
    m_rightBackEncoder.SetVelocityConversionFactor(velocityCorrection.value());

    ResetEncoders();
}
void Drivebase::Periodic() {

}
//Todo - Write code for the following 4 commands

void Drivebase::SetMotorPower(double leftPower, double rightPower){
    m_drive->TankDrive(leftPower, rightPower);
}

units::meter_t  Drivebase::GetLeftDistance(){
    //TODO: the returned value is in revolution need to fix to meters
    return units::meter_t (m_leftFrontEncoder.GetPosition());
}

units::meter_t  Drivebase::GetRightDistance(){
    //TODO: the returned value is in revolution need to fix to meters
    return units::meter_t (m_rightFrontEncoder.GetPosition());
}

units::meters_per_second_t Drivebase::GetLeftVelocity(){
    return units::meters_per_second_t(m_leftFrontEncoder.GetVelocity());
}

units::meters_per_second_t Drivebase::GetRightVelocity(){
    return units::meters_per_second_t(m_rightFrontEncoder.GetVelocity());
}

void Drivebase::ResetEncoders(){
    m_leftFrontEncoder.SetPosition(0);
    m_rightFrontEncoder.SetPosition(0);
    m_leftBackEncoder.SetPosition(0);
    m_rightBackEncoder.SetPosition(0);
}