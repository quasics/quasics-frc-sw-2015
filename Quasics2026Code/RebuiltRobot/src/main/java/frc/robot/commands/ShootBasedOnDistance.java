// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.utils.ShooterCalculator;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ShootBasedOnDistance extends Command {
  IShooter m_shooter;
  private AngularVelocity m_velocityRPM;
  private double m_kickSpeed;
  private double m_kickerDelay;
  Timer m_timer;
  ShooterCalculator m_calculator;
  IDrivebase m_drivebase;

  /** Creates a new RunShooter. */
  public ShootBasedOnDistance(IShooter shooter, IDrivebase drivebase, double kickSpeed, double kickerDelay) {
    m_shooter = shooter;
    m_drivebase = drivebase;
    m_kickSpeed = kickSpeed;
    m_kickerDelay = kickerDelay;
    m_timer = new Timer();
    m_timer.start();
    m_calculator = new ShooterCalculator();
    m_calculator.addDataPoint(70, 2700);
    m_calculator.addDataPoint(105, 3050); // half from tower to hub ish
    m_calculator.addDataPoint(140, 3300); // trench shot
    m_calculator.addDataPoint(160, 3700); // from tower/back wall
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((Subsystem) shooter, (Subsystem) drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_velocityRPM = m_calculator.getSpeedToHitHubCenter(m_drivebase.getEstimatedPose());
    m_shooter.setFlywheelRPM(m_velocityRPM);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_velocityRPM = m_calculator.getSpeedToHitHubCenter(m_drivebase.getEstimatedPose());
    m_shooter.setFlywheelRPM(m_velocityRPM);
    if (m_timer.hasElapsed(m_kickerDelay)) {
      m_shooter.setKickerSpeed(m_kickSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }
}
