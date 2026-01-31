// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.thethriftybot.devices.ThriftyNova;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ThriftyRunner extends SubsystemBase {
  private ThriftyNova motor; // motor instance

  /** Creates a new ThriftyRunner. */
  public ThriftyRunner() {
    motor = new ThriftyNova(1);
    List<ThriftyNova.Error> errors = motor.getErrors();
    for (var err : errors) {
      System.out.println(err.toString());
    }
    motor.clearErrors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motor.setPercent(1); // set motor to output full forward
  }
}
