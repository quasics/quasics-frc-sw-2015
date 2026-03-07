// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IClimber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    percentagePER = distanceExtend / 100;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_climber.setClimberSpeed(m_climberSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_climber.stopClimber();

  }

}
