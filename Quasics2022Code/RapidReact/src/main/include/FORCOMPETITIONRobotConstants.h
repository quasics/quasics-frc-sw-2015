// TO CHANGE MOTOR VALUES ON ROBOT LOOK BELOW
// TO CHANGE MOTOR VALUES ON ROBOT LOOK BELOW
// TO CHANGE MOTOR VALUES ON ROBOT LOOK BELOW
// TO CHANGE MOTOR VALUES ON ROBOT LOOK BELOW

namespace RobotValues {
  // Speed Scaling
  constexpr double TURBO_MODE_SPEED_SCALING = 0.85;
  constexpr double NORMAL_MODE_SPEED_SCALING = 0.75;
  constexpr double TURTLE_MODE_SPEED_SCALING = 0.35;

  constexpr double CLIMBER_EXTENSION_SPEED = 0.0;
  constexpr double CLIMBER_RETRACTION_SPEED =
      0.0;  // cannot Include this file into the climber subsystem

  // Intake
  constexpr double INTAKE_FORWARD_SPEED = 0.9;
  constexpr double INTAKE_BACKWARD_SPEED = -0.6;
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