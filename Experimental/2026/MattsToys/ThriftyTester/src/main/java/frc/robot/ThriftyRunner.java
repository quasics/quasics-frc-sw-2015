// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.thethriftybot.devices.ThriftyNova;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class ThriftyRunner extends SubsystemBase {
  private ThriftyNova motor; // motor instance

  /** Creates a new ThriftyRunner. */
  public ThriftyRunner() {
    motor = new ThriftyNova(7);
    List<ThriftyNova.Error> errors = motor.getErrors();
    for (var err : errors) {
      System.out.println(err.toString());
    }
    motor.clearErrors();
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      motor.setPercent(1.0);
    }
  }
}
