// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class MockClimber extends AbstractClimber {
  double leftRevolutions = 0;
  double rightRevolutions = 0;
  Timer leftRunTime = new Timer();
  Timer rightRunTime = new Timer();
  Mode leftMode = Mode.Stopped;
  Mode rightMode = Mode.Stopped;

  @Override
  protected void resetLeftEncoder_HAL() {
    leftRevolutions = 0;
  }

  @Override
  protected void resetRightEncoder_HAL() {
    rightRevolutions = 0;
  }

  @Override
  protected double getLeftRevolutions_HAL() {
    return leftRevolutions;
  }

  @Override
  protected double getRightRevolutions_HAL() {
    return rightRevolutions;
  }

  @Override
  protected void stopLeftClimber_HAL() {
    leftMode = Mode.Stopped;
    leftRunTime.stop();
    leftRunTime.reset();
  }

  @Override
  protected void stopRightClimber_HAL() {
    rightMode = Mode.Stopped;
    rightRunTime.stop();
    rightRunTime.reset();
  }

  @Override
  protected void extendLeftClimber_HAL() {
    leftMode = Mode.Extending;
    leftRunTime.restart();
  }

  @Override
  protected void extendRightClimber_HAL() {
    rightMode = Mode.Extending;
    rightRunTime.restart();
  }

  @Override
  protected void retractLeftClimber_HAL() {
    leftMode = Mode.Retracting;
    leftRunTime.restart();
  }

  @Override
  protected void retractRightClimber_HAL() {
    rightMode = Mode.Retracting;
    rightRunTime.restart();
  }

  @Override
  public void periodic() {
    // Mocked revolution rate.
    final double REVOLUTIONS_PER_SECOND = 0.5;

    // Time between updates to position while "in motion".
    final double SECONDS_PER_UPDATE = 0.1;

    // Update the revolutions, as needed.
    if (leftRunTime.hasElapsed(SECONDS_PER_UPDATE)) {
      if (leftMode == Mode.Extending) {
        leftRevolutions += REVOLUTIONS_PER_SECOND * leftRunTime.get();
        leftRunTime.reset();
      } else if (leftMode == Mode.Retracting) {
        leftRevolutions -= REVOLUTIONS_PER_SECOND * leftRunTime.get();
        leftRunTime.reset();
      }
    }
    if (rightRunTime.hasElapsed(SECONDS_PER_UPDATE)) {
      if (rightMode == Mode.Extending) {
        rightRevolutions += REVOLUTIONS_PER_SECOND * rightRunTime.get();
        rightRunTime.reset();
      } else if (rightMode == Mode.Retracting) {
        rightRevolutions -= REVOLUTIONS_PER_SECOND * rightRunTime.get();
        rightRunTime.reset();
      }
    }

    System.out.println("Left climber: " + leftRevolutions + ", Right climber: " + rightRevolutions);

    // Call the base class.
    super.periodic();
  }
}
