// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

#include <iostream>

#include "Constants.h"

// #define NOISY

Shooter::Shooter()
    : shootingMotor(CANBusIds::TalonFXIds::ShootingMotor),
      positionServo(PwmIds::ShooterServo) {
  // Settings per AndyMark docs for the L16 Actuator/servo; see:
  // https://www.andymark.com/products/actuator-l16-r-50mm-stroke-35-1-6v
  positionServo.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);

  // Make sure that the shooter starts with the servo in a known position.
  SetServoPosition(0.0);
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
#ifdef NOISY
  std::cout << "Shooter position: " << positionServo.Get() << std::endl;
#endif  // NOISY
}

void Shooter::SetSpeed(double speed) {
  shootingMotor.Set(-speed);  // Yeet the ball
}

void Shooter::Stop() {
  shootingMotor.Set(0);
}

// Also from AndyMark docs (link above):
//   For FTC:
//      To drive fully out, the position is set to 0.82.
//      To drive half way, the position is set to 0.5.
//      To drive fully in, the position is set to 0.17
//   For FRC:
//      yourActuator.setSpeed(1.0); // to open
//      yourActuator.setSpeed(-1.0);  // to close
constexpr double SERVO_RETRACTED_SPEED = -1.0;
constexpr double SERVO_EXTENDED_SPEED = +1.0;
constexpr double SERVO_POSITION_RANGE =
    SERVO_EXTENDED_SPEED - SERVO_RETRACTED_SPEED;

double Shooter::GetServoPosition() {
  auto rawPos = positionServo.GetSpeed();
  auto percentPos = (rawPos - SERVO_RETRACTED_SPEED) / SERVO_POSITION_RANGE;
  return percentPos;
}

void Shooter::SetServoPosition(double pos) {
  const double cappedPercent = std::min(1.0, std::max(pos, 0.0));
  positionServo.SetSpeed(SERVO_RETRACTED_SPEED +
                   (cappedPercent * SERVO_POSITION_RANGE));
}
