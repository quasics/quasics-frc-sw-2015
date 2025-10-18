// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.ITrivialDrivebase;

/**
 * A really simple implementation of a drive base.
 */
public class TrivialDrivebase extends SubsystemBase implements ITrivialDrivebase {

  /** Left leading motor. */
  final private SparkMax m_leftLeader = new SparkMax(Constants.QuasicsDrivebaseCanIds.LEFT_LEADER_ID,
      MotorType.kBrushless);
  /** Right leading motor. */
  final private SparkMax m_rightLeader = new SparkMax(Constants.QuasicsDrivebaseCanIds.RIGHT_LEADER_ID,
      MotorType.kBrushless);

  /**
   * WPILib differential drive implementation, providing arcade/tank drive
   * support.
   */
  final private DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  /** Creates a new TrivialDrivebase. */
  public TrivialDrivebase() {
  }

  @Override
  public void tankDrive(double leftPercentage, double rightPercentage) {
    m_differentialDrive.tankDrive(leftPercentage, rightPercentage);
  }

  @Override
  public void arcadeDrive(double leftPercentage, double rightPercentage) {
    m_differentialDrive.arcadeDrive(leftPercentage, rightPercentage);
  }
}
