// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.AbstractDrivebase;

public class TankDrive extends Command {
  final XboxController m_controller;
  final AbstractDrivebase m_drivebase;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_leftSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rightSpeedLimiter = new SlewRateLimiter(3);

  /** Creates a new TankDrive. */
  public TankDrive(AbstractDrivebase drivebase, XboxController controller) {
    m_controller = controller;
    m_drivebase = drivebase;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final boolean isSim = Robot.isSimulation();
    final double leftInput = isSim ? m_controller.getRawAxis(0) : m_controller.getLeftY();
    final double rightInput = isSim ? m_controller.getRawAxis(1) : m_controller.getRightX();

    // Get the left/right speeds. We are inverting them because Xbox controllers
    // return negative values when we push forward.
    final double leftSpeed = -m_leftSpeedLimiter.calculate(leftInput) * AbstractDrivebase.MAX_SPEED;
    final double rightSpeed = -m_rightSpeedLimiter.calculate(rightInput) * AbstractDrivebase.MAX_SPEED;

    m_drivebase.setSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }
}
