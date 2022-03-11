// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntake;

/**
 * Command to run the floor pickup subsystem.
 */
public class RunFloorPickup extends CommandBase {
  private final FloorIntake m_floorIntake;
  private final double m_speed;

  /**
   * Creates a new RunFloorPickup.
   * 
   * @param floorIntake  the floor intake/pickup subsystem
   * @param percentSpeed the speed (motor %) at which the intake should be
   *                     deployed
   */
  public RunFloorPickup(FloorIntake floorIntake, double percentSpeed) {
    m_floorIntake = floorIntake;
    m_speed = Math.max(-1, Math.min(+1, percentSpeed));

    addRequirements(m_floorIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_floorIntake.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_floorIntake.stop();
  }
}
