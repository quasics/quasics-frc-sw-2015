#pragma once

// TO CHANGE MOTOR VALUES ON ROBOT LOOK BELOW
// TO CHANGE MOTOR VALUES ON ROBOT LOOK BELOW
// TO CHANGE MOTOR VALUES ON ROBOT LOOK BELOW
// TO CHANGE MOTOR VALUES ON ROBOT LOOK BELOW

// Climber Speeds
constexpr double EXTENSION_SPEED = 1.00;

constexpr double RETRACTION_SPEED = -1.00;

namespace RobotValues {
  // Speed Scaling
  constexpr double TURBO_MODE_SPEED_SCALING = 0.85;
  constexpr double NORMAL_MODE_SPEED_SCALING = 0.75;
  constexpr double TURTLE_MODE_SPEED_SCALING = 0.35;

  // Intake
  constexpr double INTAKE_FORWARD_SPEED = 1.00;
  constexpr double INTAKE_BACKWARD_SPEED = -1.00;
  // Conveyor
  constexpr double CONVEYOR_UP_SPEED = 0.6;
  constexpr double CONVEYOR_DOWN_SPEED = -0.6;
  // Slow Shooter
  constexpr double SLOW_SHOOTER_SPEED = 0.35;
  constexpr double SLOW_SHOOTER_BACKROLLER_SPEED = 0.0;
  // Fast Shooter
  constexpr double FAST_SHOOTER_SPEED = 0.6;
  constexpr double FAST_SHOOTER_BACKROLLER_SPEED = 0.35;
  // Intake Deployment
  constexpr double EXTEND_INTAKE_SPEED = 0.5;
  constexpr double RETRACT_INTAKE_SPEED = -0.5;
}  // namespace RobotValues