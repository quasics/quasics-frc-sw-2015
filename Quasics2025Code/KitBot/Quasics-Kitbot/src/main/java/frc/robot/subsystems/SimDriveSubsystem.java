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
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// Class to drive the robot over Sim
// TODO: Extend AbstractDrivebase instead of SubsystemBase
public class SimDriveSubsystem extends SubsystemBase {
  private final Field2d fieldSim = new Field2d();
  private final PWMSparkMax m_leftLeader;
  private final PWMSparkMax m_leftFollower;
  private final PWMSparkMax m_rightLeader;
  private final PWMSparkMax m_rightFollower;

  private final DifferentialDrive drive;

  // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/simulation-instance.html
  private AnalogGyro m_gyro = new AnalogGyro(1);
  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  //TODO: Update to use these
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;

  // TODO: Update to use these
  private final DifferentialDrivetrainSim m_driveSim;
  private final Field2d m_fieldSim = new Field2d();
  private static final int kEncoderResolutionTicksPerRevolution = -4096;

  // TODO: Create DifferentialDriveOdometry
  // 1. How to create:
  //   docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html
  // 2. How to use:
  //   docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/odometry-simgui.html

  public SimDriveSubsystem(RobotSettings.Robot robot) {

    // create PWMSparkMax motors
    m_leftLeader = new PWMSparkMax(SimDriveConstants.LEFT_LEADER_ID);
    m_leftFollower = new PWMSparkMax(SimDriveConstants.LEFT_FOLLOWER_ID);
    m_rightLeader = new PWMSparkMax(SimDriveConstants.RIGHT_LEADER_ID);
    m_rightFollower = new PWMSparkMax(SimDriveConstants.RIGHT_FOLLOWER_ID);

    // Encoders tell us where the motors are, so that we can estimate position and velocity
    // docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/simulation-instance.html
    m_leftEncoder = new Encoder(0, 1);
    m_rightEncoder = new Encoder(2, 3);

    // TODO: Set up the 'DistancePerPulse' so that we can calculate how far we move based on encoder data:
    //   docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/updating-drivetrain-model.html


    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    // Gyros tell us the current angular velocity of the robot
    // Extra: Gyros drift a LOT over time, so you cannot just "trust it". There's lots of
    // internal fancy math that the libraries do for us to help filter this, which we can talk about one day!
    // TODO: Create a gyroscope simulation so that we know our heading
    //    docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/simulation-instance.html#simulating-gyroscopes

    drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
    m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
      KitbotGearing.k10p71,        // 10.71:1
      KitbotWheelSize.kSixInch,    // 6" diameter wheels.
    null                         // No measurement noise.
    );

    SmartDashboard.putData("field", m_fieldSim);
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    // TODO: Look at setSpeeds_HAL and AbstractDriveBase arcadeDrive as an example
    drive.arcadeDrive(xSpeed, zRotation);
  }


  // docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/odometry-simgui.html
  @Override
  public void periodic() {
    // TODO: get the simulated sensor readings and use them to update the odometry and field sim.
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_driveSim.setInputs(m_leftLeader.get() * RobotController.getInputVoltage(),
        m_rightLeader.get() * RobotController.getInputVoltage());

    // Simulated clock ticks forward
    m_driveSim.update(0.02);

    // Update the encoders and gyro, based on what the drive train simulation says
    // happend.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  }

}
