// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IClimber;

public class ClimberForPercentage extends Command {

  IClimber m_climber;
  private double m_climberSpeed;
  private final double distanceExtend = 30; // Maximum height; Change as needed ************************
  private double percentagePER; // amount of value each percentage has (percebtagePER = distanceExtend / 100)
  private int loops; // how many loops the thing has done

  // private boolean m_forward; Probably don't need

  /** Creates a new RunIntake. */
  public ClimberForPercentage(IClimber climber, double climberSpeed) {
    m_climber = climber;
    m_climberSpeed = climberSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((SubsystemBase) climber);
  }

  @Override
  public void initialize() {
    percentagePER = distanceExtend / 100;
  }

  @Override
  public void execute() {
    m_climber.setClimberSpeed(m_climberSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stopClimber();
  }

}
