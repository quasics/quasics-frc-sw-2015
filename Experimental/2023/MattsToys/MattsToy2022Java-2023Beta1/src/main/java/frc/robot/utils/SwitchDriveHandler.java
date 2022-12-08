// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class SwitchDriveHandler {

  private final DrivePowerSupplier leftStick;
  private final DrivePowerSupplier rightStick;
  private boolean isFlipped = false;

  public SwitchDriveHandler(DrivePowerSupplier leftStick, DrivePowerSupplier rightStick) {
    this.leftStick = leftStick;
    this.rightStick = rightStick;
  }

  public void switchDirections() {
    isFlipped = !isFlipped;
  }

  public DrivePowerSupplier getLeftSupplier() {
    return () -> {
      if (isFlipped) {
        return -rightStick.get();
      } else {
        return leftStick.get();
      }
    };
  }

  public DrivePowerSupplier getRightSupplier() {
    return () -> {
      if (isFlipped) {
        return -leftStick.get();
      } else {
        return rightStick.get();
      }
    };
  }
}
