// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//IMPORTANT!!!! This ENTIRE COMMAND is copy-pasted from the training robot! It almost certainly needs a METRIC TON OF DEBUGGING!!
//I've started, but I have no idea what I'm doing :( 
//-Robert

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.sensors.IGyro;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TurnForDegrees extends Command {
  final private AbstractDrivebase m_drivebase;
  final double m_rotationInDegrees;
  final double m_speed;
  Angle m_stopAngle;

  /**
   * Creates a new TurnForDegrees.
   *
   * @param drivebase         The drivebase subsystem on which this command will
   *                          run.
   * @param rotationInDegrees The number of degrees to rotate.
   * @param rotationalSpeed   The speed at which to rotate (as a % of motor
   *                          speed, -1.0 to +1.0).
   */
  public TurnForDegrees(AbstractDrivebase drivebase, double rotationInDegrees, double speed) {
    m_drivebase = drivebase;
    m_rotationInDegrees = rotationInDegrees;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Figure out when to stop (before we start moving).
    Angle startingDirection = m_drivebase.getHeading();
    Angle rotationAngle = Units.Degrees.of(m_rotationInDegrees);
    m_stopAngle = startingDirection.plus(rotationAngle);

    // With a tank drive, you can rotate by driving the left and right sides in
    // opposite directions. If the speeds are the same magnitude, the robot will
    // rotate in place. (The sign of the speed determines the direction of the
    // turn.)
    //
    // With a robot that has "arcade drive", you can just set the speed of the
    // rotation, and the forward speed to 0, for the same results.
    if (m_rotationInDegrees > 0) {
      m_drivebase.setSpeeds(m_speed, -m_speed);
    } else {
      m_drivebase.setSpeeds(-m_speed, m_speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Rotation in degrees: " + m_rotationInDegrees);
    System.out.println("Stop angle: " + m_stopAngle.in(Degrees));
    if (m_rotationInDegrees > 0) {
      if (m_drivebase.getHeading().gte(m_stopAngle)) {
        return true;
      } else {
        return false;
      }
    } else {
      if (m_drivebase.getHeading().lte(m_stopAngle)) {
        return true;
      } else {
        return false;
      }
    }
  }
}