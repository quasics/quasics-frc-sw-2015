// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimDriveConstants;
import frc.robot.utils.RobotSettings;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// Class to drive the robot over Sim
public class SimDriveSubsystem extends SubsystemBase {
  private final Field2d fieldSim = new Field2d();
  private final PWMSparkMax m_leftLeader;
  private final PWMSparkMax m_leftFollower;
  private final PWMSparkMax m_rightLeader;
  private final PWMSparkMax m_rightFollower;

  private final DifferentialDrive drive;

  public SimDriveSubsystem(RobotSettings.Robot robot) {
    // create PWMSparkMax motors
    m_leftLeader = new PWMSparkMax(SimDriveConstants.LEFT_LEADER_ID);
    m_leftFollower = new PWMSparkMax(SimDriveConstants.LEFT_FOLLOWER_ID);
    m_rightLeader = new PWMSparkMax(SimDriveConstants.RIGHT_LEADER_ID);
    m_rightFollower = new PWMSparkMax(SimDriveConstants.RIGHT_FOLLOWER_ID);

    // m_leftEncoder = new Encoder(...);
    // m_rightEncoder = new Encoder(...);

    DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim(
      KitbotMotor.kDualCIMPerSide,
      KitbotGearing.k10p71,
      KitbotWheelSize.kSixInch,
      null
    );

    // SmartDashboard.putData("field", m_fieldSim);
  }

  @Override
  public void periodic() {
  // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.
  m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
  m_rightMotor.get() * RobotController.getInputVoltage());
// Advance the model by 20 ms. Note that if you are running this
// subsystem in a separate thread or have changed the nominal timestep
// of TimedRobot, this value needs to match it.
m_driveSim.update(0.02);
// Update all of our sensors.
m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

}
