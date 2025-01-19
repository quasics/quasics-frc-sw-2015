// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.simulations.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.IGyro;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.utils.BulletinBoard;

public class SimDrivebase extends SubsystemBase implements IDrivebase {
  // Control ports for our drive motors. (These would be specific to a given
  // robot.)
  public static final int LEFT_DRIVE_PWM_ID = 0;
  public static final int RIGHT_DRIVE_PWM_ID = 1;
  public static final int LEFT_DRIVE_ENCODER_PORT_A = 0;
  public static final int LEFT_DRIVE_ENCODER_PORT_B = 1;
  public static final int RIGHT_DRIVE_ENCODER_PORT_A = 2;
  public static final int RIGHT_DRIVE_ENCODER_PORT_B = 3;

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
      kRobotTrackWidthMeters.baseUnitMagnitude());

  // Hardware allocation
  private final PWMSparkMax m_left = new PWMSparkMax(LEFT_DRIVE_PWM_ID);
  private final PWMSparkMax m_right = new PWMSparkMax(RIGHT_DRIVE_PWM_ID);
  private final Encoder m_leftEncoder = new Encoder(LEFT_DRIVE_ENCODER_PORT_A, LEFT_DRIVE_ENCODER_PORT_B);
  private final Encoder m_rightEncoder = new Encoder(RIGHT_DRIVE_ENCODER_PORT_A, RIGHT_DRIVE_ENCODER_PORT_B);
  private final AnalogGyroSim m_gyroSim;
  private final IGyro m_wrappedGyro;

  /** Odometry for the robot, purely calculated from encoders/gyro. */
  final private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0,
      new Pose2d());

  // Objects used in simulation mode.
  final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,
      DCMotor.getCIM(2), 8,
      kRobotTrackWidthMeters.baseUnitMagnitude(), kWheelRadiusMeters.baseUnitMagnitude(), null);
  final Field2d m_fieldSim = new Field2d();

  /** Creates a new SimDrivebase. */
  public SimDrivebase() {
    setName(NAME);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.setInverted(true);

    // Set the distance per pulse (in meters) for the drive encoders. We can simply
    // use the distance traveled for one rotation of the wheel divided by the
    // encoder resolution.
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadiusMeters.baseUnitMagnitude()
        / kEncoderResolutionTicksPerRevolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadiusMeters.baseUnitMagnitude()
        / kEncoderResolutionTicksPerRevolution);

    // Make sure our encoders are zeroed out on startup.
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    // Set up the gyro
    final AnalogGyro rawGyro = new AnalogGyro(0);
    m_wrappedGyro = IGyro.wrapGyro(rawGyro);

    //
    // Pure simulation support
    //
    m_gyroSim = new AnalogGyroSim(rawGyro);

    // Add the simulated field to the smart dashboard
    SmartDashboard.putData("Field", m_fieldSim);
  }

  @Override
  public void periodic() {
    super.periodic();

    updateOdometry();

    // Update published field simulation data. We're doing this here (in the
    // periodic function, rather than in the simulationPeriodic function) because we
    // want to take advantage of the fact that the odometry has just been updated.
    //
    // When we move stuff into a base class, the code above would be there, and this
    // would be in the overridden periodic function for this (simulation-specific)
    // class.
    var pose = getOdometry().getPoseMeters();
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
    m_fieldSim.setRobotPose(pose);

    // m_fieldSim.getObject("Estimated pose").setPose(getEstimatedPose());

    // Share the current pose with other subsystems (e.g., vision).
    BulletinBoard.common.updateValue(POSITION_KEY, pose);
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(m_left.get() * RobotController.getInputVoltage(),
        m_right.get() * RobotController.getInputVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(0.02);

    // Update the encoders, based on what the drive train simulation says happend.
    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());

    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    // Publish the data for any that need it.
    // BulletinBoard.common.updateValue(SIMULATOR_POSE_KEY,
    // m_drivetrainSimulator.getPose());
  }

  @Override
  public void setSpeed(double leftPercentage, double rightPercentage) {
    // Clamp speeds to the range [-1.0, 1.0].
    leftPercentage = Math.max(-1.0, Math.min(1.0, leftPercentage));
    rightPercentage = Math.max(-1.0, Math.min(1.0, rightPercentage));

    m_left.set(leftPercentage);
    m_right.set(rightPercentage);
  }

  public Distance getLeftPositionMeters() {
    return Meters.of(m_leftEncoder.getDistance());
  }

  public Distance getRightPositionMeters() {
    return Meters.of(m_rightEncoder.getDistance());
  }

  // TODO: Move to a base class as a protected method. (It shouldn't be exposed as
  // public, since we don't want client code to directly manipulate/change the
  // data.)
  protected final DifferentialDriveOdometry getOdometry() {
    return m_odometry;
  }

  /** Get the current robot pose, based on odometery. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public Angle getHeading() {
    return m_wrappedGyro.getAngle();
  }

  /**
   * Update the robot's odometry.
   * 
   * TODO: This should be moved to a base class or interface.
   */
  public void updateOdometry() {
    final Rotation2d rotation = m_wrappedGyro.getRotation2d();
    final double leftDistanceMeters = m_leftEncoder.getDistance();
    final double rightDistanceMeters = m_rightEncoder.getDistance();
    m_odometry.update(rotation, leftDistanceMeters, rightDistanceMeters);
    // m_poseEstimator.update(rotation, leftDistanceMeters.in(Meters),
    // rightDistanceMeters.in(Meters));
  }

  /**
   * Resets robot odometry (e.g., if we know that we've been placed at a
   * specific position/angle on the field, such as at the start of a match).
   */
  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_odometry.resetPosition(m_wrappedGyro.getRotation2d(), 0, 0, pose);
    // m_poseEstimator.resetPosition(m_wrappedGyro.getRotation2d(), 0, 0, pose);

    // Update the pose information in the simulator.
    m_drivetrainSimulator.setPose(pose);
  }
}
