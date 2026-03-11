// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.utils.ShooterCalculator;

public class ShootBasedOnDistance extends Command {
  private final IShooter m_shooter;
  private final IDrivebase m_drivebase;

  private final double m_kickSpeed;
  private final double m_kickerDelay;
  private final Timer m_timer = new Timer();;
  private final ShooterCalculator m_calculator = new ShooterCalculator();

  /** Creates a new RunShooter. */
  public ShootBasedOnDistance(IShooter shooter, IDrivebase drivebase, double kickSpeed, double kickerDelay) {
    m_shooter = shooter;
    m_drivebase = drivebase;
    m_kickSpeed = kickSpeed;
    m_kickerDelay = kickerDelay;

    // Load our spot measurements into the shot calculator.
    m_calculator.addDataPoint(Inches.of(70), RPM.of(2700));
    m_calculator.addDataPoint(Inches.of(105), RPM.of(3050)); // half from tower to hub ish
    m_calculator.addDataPoint(Inches.of(140), RPM.of(3300)); // trench shot
    m_calculator.addDataPoint(Inches.of(160), RPM.of(3700)); // from tower/back wall

    // FINDME(Rylie): Note that you can actually get away without declaring that you
    // use the drivebase, since you're only treating it as a read-only source of
    // data (i.e., pose estimates), and not trying to tell it to *do* anything. It's
    // not *wrong* to list it here, but it means that you'd be unable to have code
    // that can shoot *while moving*.
    addRequirements((Subsystem) shooter, (Subsystem) drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final AngularVelocity velocityRPM = m_calculator.getSpeedToHitHubCenter(m_drivebase.getEstimatedPose());
    m_shooter.setFlywheelRPM(velocityRPM);
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final AngularVelocity velocityRPM = m_calculator.getSpeedToHitHubCenter(m_drivebase.getEstimatedPose());
    m_shooter.setFlywheelRPM(velocityRPM);
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
