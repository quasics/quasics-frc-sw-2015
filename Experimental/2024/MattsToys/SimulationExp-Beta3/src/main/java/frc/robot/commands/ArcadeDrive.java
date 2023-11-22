// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.AbstractDrivebase;

public class ArcadeDrive extends Command {
  XboxController m_controller;
  AbstractDrivebase m_drivebase;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(AbstractDrivebase drivebase, XboxController controller) {
    m_controller = controller;
    m_drivebase = drivebase;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivebase.arcadeDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final boolean isSim = Robot.isSimulation();
    final double xInput = isSim ? m_controller.getRawAxis(0) : m_controller.getLeftY();
    final double rotInput = isSim ? m_controller.getRawAxis(1) : m_controller.getRightX();

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = -m_speedLimiter.calculate(xInput) * AbstractDrivebase.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = -m_rotLimiter.calculate(rotInput) * AbstractDrivebase.MAX_ANGULAR_SPEED;

    m_drivebase.arcadeDrive(xSpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
