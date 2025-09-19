// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.SimDriveConstants;
import frc.robot.utils.RobotSettings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Class to drive the robot over Sim
public class SimDriveSubsystem extends AbstractDrivebase {
  private final Field2d fieldSim = new Field2d();
  private final PWMSparkMax m_leftLeader;
  private final PWMSparkMax m_leftFollower;
  private final PWMSparkMax m_rightLeader;
  private final PWMSparkMax m_rightFollower;

  private final DifferentialDrive drive;
  // TODO: Update to use this
  private final AnalogGyroSim m_gyroSim;
  private final AnalogGyro m_gyro;

  //TODO: Update to use these
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;

  private final DifferentialDrivetrainSim m_driveSim;
  private final Field2d m_fieldSim = new Field2d();
  private static final int kEncoderResolutionTicksPerRevolution = -4096;

  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
      new Pose2d());

  public SimDriveSubsystem(RobotSettings.Robot robot) {

    // create PWMSparkMax motors
    m_leftLeader = new PWMSparkMax(SimDriveConstants.LEFT_LEADER_ID);
    m_leftFollower = new PWMSparkMax(SimDriveConstants.LEFT_FOLLOWER_ID);
    m_rightLeader = new PWMSparkMax(SimDriveConstants.RIGHT_LEADER_ID);
    m_rightFollower = new PWMSparkMax(SimDriveConstants.RIGHT_FOLLOWER_ID);

    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/simulation-instance.html
    m_leftEncoder = new Encoder(0, 1);
    m_rightEncoder = new Encoder(2, 3);

    m_leftEncoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / kEncoderResolutionTicksPerRevolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6) / kEncoderResolutionTicksPerRevolution);

    m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    m_leftEncoderSim = new EncoderSim(m_rightEncoder);

    m_gyro = new AnalogGyro(1);
    m_gyroSim = new AnalogGyroSim(m_gyro);

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

  @Override
  // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/odometry-simgui.html
  public void periodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    m_odometry.update(m_gyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
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
