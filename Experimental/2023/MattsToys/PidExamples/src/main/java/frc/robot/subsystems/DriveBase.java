// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  private final CANSparkMax m_leftFront = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_FRONT_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private final CANSparkMax m_leftRear = new CANSparkMax(Constants.MotorIds.SparkMax.LEFT_REAR_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private final CANSparkMax m_rightFront = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_FRONT_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private final CANSparkMax m_rightRear = new CANSparkMax(Constants.MotorIds.SparkMax.RIGHT_REAR_DRIVE_MOTOR_ID,
      MotorType.kBrushless);

  private final SparkMaxPIDController m_leftController;
  private final SparkMaxPIDController m_rightController;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  /** Creates a new DriveBase. */
  public DriveBase() {
    final double GEAR_RATIO = Constants.DRIVE_BASE_GEAR_RATIO_SALLY;
    final double WHEEL_CIRCUMFERENCE = Math.PI * Constants.WHEEL_DIAMETER_METERS;

    m_rightRear.follow(m_rightFront);
    m_leftRear.follow(m_leftFront);

    m_leftController = m_leftFront.getPIDController();
    m_rightController = m_rightFront.getPIDController();

    m_leftEncoder = m_leftFront.getEncoder();
    m_rightEncoder = m_rightFront.getEncoder();

    m_leftEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE / GEAR_RATIO);
    m_rightEncoder.setPositionConversionFactor(WHEEL_CIRCUMFERENCE / GEAR_RATIO);

    m_leftEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE / GEAR_RATIO);
    m_rightEncoder.setVelocityConversionFactor(WHEEL_CIRCUMFERENCE / GEAR_RATIO);


    // simulation initiation
    if (Robot.isSimulation()) {
      // add sparks to physics simulator
      REVPhysicsSim.getInstance().addSparkMax(m_leftFront, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_leftRear, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_rightFront, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_rightRear, DCMotor.getNEO(1));
    }
  }

  public void stop() {
    m_leftController.setReference(0, ControlType.kDutyCycle);
    m_rightController.setReference(0, ControlType.kDutyCycle);
    m_rightFront.stopMotor();
    m_leftFront.stopMotor();
  }

  public void tankDrive(double leftPercent, double rightPercent) {
    double leftVelocity = leftPercent * Constants.MAX_SPEED_METERS_PER_SEC;
    double rightVelocity = rightPercent * Constants.MAX_SPEED_METERS_PER_SEC;

    m_leftController.setReference(leftVelocity, ControlType.kDutyCycle);
    m_rightController.setReference(rightVelocity, ControlType.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
