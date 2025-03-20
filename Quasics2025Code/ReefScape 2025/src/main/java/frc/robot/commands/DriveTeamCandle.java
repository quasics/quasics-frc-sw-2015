// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.ICandle;
import frc.robot.subsystems.drivebase.AbstractDrivebase;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTeamCandle extends Command {
  /** Creates a new DriveTeamCandle. */
  private final Candle m_candle;
  private final AbstractDrivebase m_drivebase;

  private final double CORRECT_X = 7.586;
  private final double ERROR = 0.1;
  private final double FIELD_LENGTH = 17.548;

  public DriveTeamCandle(Candle candle, AbstractDrivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_candle = candle;
    m_drivebase = drivebase;
    addRequirements(m_candle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var optAlliance = DriverStation.getAlliance();
    DriverStation.Alliance alliance = optAlliance.orElse(null);
    Color8Bit color = Candle.BLACK; // default to black

    Pose2d pose = m_drivebase.getPose();
    double currentX = pose.getX();
    double correctX = (alliance == Alliance.Blue) ? CORRECT_X : FIELD_LENGTH - CORRECT_X;

    if (alliance == Alliance.Blue) {
      if (Math.abs(currentX - correctX) < ERROR) {
        color = Candle.GREEN;
      } else {
        color = Candle.BLUE;
      }
    } else if (alliance == Alliance.Red) {
      if (Math.abs(currentX - correctX) < ERROR) {
        color = Candle.GREEN;
      } else {
        color = Candle.RED;
      }
    }

    if (DriverStation.isEnabled()) {
      m_candle.setIntensity(1.0);
    } else {
      m_candle.setIntensity(0.1);
    }

    m_candle.setColor(color);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
