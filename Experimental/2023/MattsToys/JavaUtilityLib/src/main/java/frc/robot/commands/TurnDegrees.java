// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.AbstractDriveBase;
import frc.robot.subsystems.DriveBaseInterface;

public class TurnDegrees extends CommandBase {
  private final DriveBaseInterface m_drive;
  private final double m_degrees;
  private final double m_speed;
  private double m_targetAngle = 0;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegrees(double speed, double degrees, DriveBaseInterface drive) {
    // TODO(mjh): Update this code to handle +/- degrees and speeds correctly.
    m_degrees = degrees;
    m_speed = speed;
    m_drive = drive;
    addRequirements((Subsystem) drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read gyro value for starting point
    m_drive.arcadeDrive(0, 0);
    m_targetAngle = m_drive.getYawDegrees() + m_degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    final double kCloseEnough = 0.25; // within this many degrees?  We're OK.
    // TODO(mjh): Replace "kCloseEnough" with a c-tor parameter.

    final double currentAngleDegrees = m_drive.getYawDegrees();
    if (m_degrees < 0 && currentAngleDegrees <= (m_targetAngle + kCloseEnough)) {
      return true;
    } else if (m_degrees > 0 && currentAngleDegrees >= (m_targetAngle - kCloseEnough)) {
      return true;
    } else {
      return false;
    }
    /*
     * We could also convert distance travelled to degrees. The Standard
     * Romi Chassis found here,
     * https://www.pololu.com/category/203/romi-chassis-kits,
     * has a wheel placement diameter (149 mm) - width of the wheel (8 mm) = 141 mm
     * or 5.551 inches. We could then take into consideration the width of the tires:
     *
     *    // Compare distance traveled from start to distance based on degree turn
     *    final double distanceTraveledInFullTurn = Math.PI * getWheelPlacementDiameterInch();
     *    final double inchPerDegree = distanceTraveledInFullTurn / 360;
     *    return getAverageTurningDistance() >= (inchPerDegree * m_degrees);
     */
  }

  protected double getAverageTurningDistance() {
    // Note that this assumes that the encoders were reset at the beginning.
    // A better approach would be to capture the initial readings, and then
    // compute offsets.
    double leftDistance = Math.abs(m_drive.getLeftEncoderPositionMeters());
    double rightDistance = Math.abs(m_drive.getLeftEncoderPositionMeters());
    return (leftDistance + rightDistance) / 2.0;
  }

  protected final double getWheelPlacementDiameterInch() {
    return m_drive.getLeftEncoderPositionMeters() / AbstractDriveBase.MILLIMETERS_PER_INCH;
  }
}
