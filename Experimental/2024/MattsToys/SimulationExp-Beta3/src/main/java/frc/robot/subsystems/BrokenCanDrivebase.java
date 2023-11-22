// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;

public class BrokenCanDrivebase extends AbstractDrivebase {
  public static final double MAX_SPEED = 3.0; // in meters/sec
  public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation/sec

  // Motor IDs are based on those Quasics has used over the last couple of years.
  static final int LEFT_FRONT_CAN_ID = 1;
  static final int LEFT_REAR_CAN_ID = 2;
  static final int RIGHT_FRONT_CAN_ID = 3;
  static final int RIGHT_REAR_CAN_ID = 4;
  static final double ANDYMARK_6IN_PLACTION_DIAMETER_METERS = Units.inchesToMeters(6.0);
  static final double GLADYS_GEAR_RATIO = 8.45;
  static final double TRACK_WIDTH_METERS = 0.381 * 2;
  static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * ANDYMARK_6IN_PLACTION_DIAMETER_METERS;
  static final double DISTANCE_SCALING_FACTOR_FOR_GEARING = WHEEL_CIRCUMFERENCE_METERS / GLADYS_GEAR_RATIO;
  static final double VELOCITY_SCALING_FACTOR = DISTANCE_SCALING_FACTOR_FOR_GEARING / 60;

  private final AnalogGyro m_gyro = new AnalogGyro(0);
  private final IGyro m_wrappedGyro = IGyro.wrapAnalogGyro(m_gyro);

  final CANSparkMax m_leftRear = new CANSparkMax(LEFT_REAR_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_rightRear = new CANSparkMax(RIGHT_REAR_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_leftFront = new CANSparkMax(LEFT_FRONT_CAN_ID, MotorType.kBrushless);
  final CANSparkMax m_rightFront = new CANSparkMax(RIGHT_FRONT_CAN_ID, MotorType.kBrushless);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftRear, m_leftFront);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightRear, m_rightFront);

  private final RelativeEncoder m_leftEncoder = m_leftRear.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightRear.getEncoder();

  private final TrivialEncoder m_leftTrivialEncoder = new SparkMaxEncoderWrapper(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = new SparkMaxEncoderWrapper(m_rightEncoder);

  // PID controllers are used to generate voltages, based on targeted speeds (in
  // the range [-1,+1]).
  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

  // Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // Simulation classes help us simulate our robot
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
      // Note: these constants come from SysID, and should be calculated for the
      // robot!
      /* kVLinear = */ 1.98,
      /* kALinear = */ 0.2,
      /* kVAngular = */ 1.5,
      /* kAAngular = */ 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(
      m_drivetrainSystem,
      DCMotor.getNEO(2), // Just need to simulate the left and right motor groups
      GLADYS_GEAR_RATIO,
      TRACK_WIDTH_METERS,
      /* wheelRadiusMeters = */ ANDYMARK_6IN_PLACTION_DIAMETER_METERS / 2,
      null);
  // Note: the EncoderSim class doesn't work with CANSparkMax encoders, since they
  // aren't "Encoder" objects. :-(
  // private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  // private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  /** Creates a new Drivebase. */
  public BrokenCanDrivebase() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.setInverted(true);
    m_leftFront.setIdleMode(IdleMode.kBrake);
    m_leftRear.setIdleMode(IdleMode.kBrake);
    m_rightFront.setIdleMode(IdleMode.kBrake);
    m_rightRear.setIdleMode(IdleMode.kBrake);

    ////////////////////////////////////////
    // Configure the encoders.

    System.out.println("Wheel circumference (m): " + WHEEL_CIRCUMFERENCE_METERS);

    // Conversion factor from units in rotations (or RPM) to meters (or m/s).
    System.out.println("Using gear ratio: " + GLADYS_GEAR_RATIO);
    System.out.println("Adjustment for gearing (m/rotation): " + DISTANCE_SCALING_FACTOR_FOR_GEARING);
    System.out.println("Velocity adj.: " + VELOCITY_SCALING_FACTOR);

    m_leftEncoder.setPositionConversionFactor(DISTANCE_SCALING_FACTOR_FOR_GEARING);
    m_rightEncoder.setPositionConversionFactor(DISTANCE_SCALING_FACTOR_FOR_GEARING);

    m_leftEncoder.setVelocityConversionFactor(VELOCITY_SCALING_FACTOR);
    m_rightEncoder.setVelocityConversionFactor(VELOCITY_SCALING_FACTOR);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    ////////////////////////////////////////
    // Finish simulation setup.

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(m_leftRear, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_rightRear, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_leftFront, DCMotor.getNEO(1));
      REVPhysicsSim.getInstance().addSparkMax(m_rightFront, DCMotor.getNEO(1));

      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Compute the basic "feedforward" value needed for the target speeds.
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    // Compute the voltages we'll need for the desired speeds.
    double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getVelocity(),
        speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getVelocity(),
        speeds.rightMetersPerSecond);

    System.out.println("leftOutput = " + leftOutput + ", rightOutput = " + rightOutput);

    // Update the motors.
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis (in m/s)
   * @param rot    the rotation
   */
  public void arcadeDrive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    updateOdometry();

    var pose = m_odometry.getPoseMeters();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    m_fieldSim.setRobotPose(pose);
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        m_leftGroup.get() * RobotController.getInputVoltage(),
        m_rightGroup.get() * RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.02);

    m_leftEncoder.setPosition(m_drivetrainSimulator.getLeftPositionMeters());
    // m_leftEncoder.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoder.setPosition(m_drivetrainSimulator.getRightPositionMeters());
    // m_rightEncode.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  protected DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  protected TrivialEncoder getLeftEncoder() {
    return m_leftTrivialEncoder;
  }

  protected TrivialEncoder getRightEncoder() {
    return m_rightTrivialEncoder;
  }

  protected IGyro getGyro() {
    return m_wrappedGyro;
  }
}
