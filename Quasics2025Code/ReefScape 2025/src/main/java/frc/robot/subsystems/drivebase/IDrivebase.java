// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotSettings;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class IDrivebase extends SubsystemBase {

  private final DifferentialDriveKinematics m_kinematics;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final Distance m_driveBaseLengthWithBumpers;
  private final Distance m_driveBaseWidthWithBumpers;

  /** Creates a new IDrivebase. */
  public IDrivebase(RobotSettings.Robot robot) {
    this(robot.trackWidthMeters);
    super.setName(robot.name());
  }

  protected IDrivebase(Distance trackWidthMeters) {
    m_kinematics = new DifferentialDriveKinematics(trackWidthMeters);
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_kinematics, new Rotation2d(),
     0, 
     0, new Pose2d());
   
    // TODO: Move drive base dimensions into new data from the subclasses
    m_driveBaseLengthWithBumpers = Inches.of(29);
    m_driveBaseWidthWithBumpers = Inches.of(26);
  }

  public Distance getLengthIncludingBumpers() {
    return m_driveBaseLengthWithBumpers;
  }

  public Distance getWidthIncludingBumpers() {
    return m_driveBaseWidthWithBumpers;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
