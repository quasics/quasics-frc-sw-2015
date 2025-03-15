// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.abstracts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ISingleJointArm;

public abstract class AbstractSparkMaxArm extends SubsystemBase implements ISingleJointArm {

  /** Motor controller running the arm. */
  protected SparkMax m_motorController;

  /** Reference/target position for arm. (Saved for logging purposes.) */
  protected Angle m_referencePosition = null;

  /** Creates a new AbstractSparkMaxArm. */
  protected AbstractSparkMaxArm(int motorCanId) {
    setName(SUBSYSTEM_NAME);

    m_motorController = new SparkMax(motorCanId, MotorType.kBrushless);
  }
}
