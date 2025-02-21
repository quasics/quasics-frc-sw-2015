// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.SparkMaxEncoderWrapper;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.utils.RobotSettings;

public class SimulationDrivebase extends AbstractDrivebase {
  private static final Distance kWheelRadius = Meters.of(0.0508);
  private static final int kEncoderResolutionTicksPerRevolution = -4096;

  private final IGyro m_wrappedGyro;

  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSim;
  private final Field2d m_fieldSim = new Field2d();
  private final AnalogGyroSim m_gyroSim;

  /*
   * final PWMSparkMax m_leftLeader = new
   * PWMSparkMax(SimulationPorts.LEFT_FRONT_DRIVE_PWM_ID);
   * final PWMSparkMax m_leftFollower = new
   * PWMSparkMax(SimulationPorts.LEFT_REAR_DRIVE_PWM_ID);
   * final PWMSparkMax m_rightLeader = new
   * PWMSparkMax(SimulationPorts.RIGHT_FRONT_DRIVE_PWM_ID);
   * final PWMSparkMax m_rightFollower = new
   * PWMSparkMax(SimulationPorts.RIGHT_REAR_DRIVE_PWM_ID);
   */

  final SparkMax m_leftLeaderSpark = new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless);
  final SparkMax m_rightLeaderSpark =
      new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless);
  final DCMotor m_leftGearBox = DCMotor.getNEO(2);
  final DCMotor m_rightGearBox = DCMotor.getNEO(2);

  final SparkMaxConfig m_leftLeaderConfig = new SparkMaxConfig();
  final SparkMaxConfig m_rightLeaderConfig = new SparkMaxConfig();

  final SparkMaxSim m_leftLeaderSparkSim = new SparkMaxSim(m_leftLeaderSpark, m_leftGearBox);
  final SparkMaxSim m_rightLeaderSparkSim = new SparkMaxSim(m_rightLeaderSpark, m_rightGearBox);

  private final RelativeEncoder m_leftEncoder = m_leftLeaderSpark.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeaderSpark.getEncoder();
  private final TrivialEncoder m_leftTrivialEncoder = new SparkMaxEncoderWrapper(m_leftEncoder);
  private final TrivialEncoder m_rightTrivialEncoder = new SparkMaxEncoderWrapper(m_rightEncoder);

  /*
   * private final EncoderSim m_leftEncoderSim;
   * private final EncoderSim m_rightEncoderSim;
   */

  public static final Distance TRACK_WIDTH_METERS = Meters.of(Constants.SallyConstants.TRACK_WIDTH);
  public static final Distance WHEEL_CIRCUMFERENCE = Inches.of(6 * Math.PI);
  public static final double GEAR_RATIO = 8.45;

  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase(RobotSettings.Robot robot) {
    super(RobotSettings.Robot.Simulator);
    super.setName(getClass().getSimpleName());

    final AnalogGyro rawGyro = new AnalogGyro(0);
    m_wrappedGyro = IGyro.wrapGyro(rawGyro);

    // configureDriveMotorsAndSensors();

    m_drivetrainSim = new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2),
        GEAR_RATIO, robot.trackWidthMeters.in(Meters), kWheelRadius.in(Meters), null);
    m_gyroSim = new AnalogGyroSim(rawGyro);
    /*
     * m_leftEncoderSim = new EncoderSim(m_leftEncoder);
     * m_rightEncoderSim = new EncoderSim(m_rightEncoder);
     */

    /*
     * m_leftTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder,
     * m_leftEncoderSim);
     * m_rightTrivialEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder,
     * m_rightEncoderSim);
     */

    // Set the distance per pulse (in meters) for the drive encoders. We can simply
    // use the distance traveled for one rotation of the wheel divided by the
    // encoder resolution.

    // distancePerPulse = 2 * Math.PI * kWheelRadius.in(Meters) /
    // kEncoderResolutionTicksPerRevolution;

    final double distanceScalingFactorForGearing = WHEEL_CIRCUMFERENCE.div(GEAR_RATIO).in(Meters);
    final double velocityScalingFactor = distanceScalingFactorForGearing / 60;

    m_leftLeaderConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    m_leftLeaderConfig.encoder.velocityConversionFactor(velocityScalingFactor);
    m_rightLeaderConfig.encoder.positionConversionFactor(distanceScalingFactorForGearing);
    m_rightLeaderConfig.encoder.velocityConversionFactor(velocityScalingFactor);

    // Make sure our encoders are zeroed out on startup.
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    SmartDashboard.putData("field", m_fieldSim);
  }

  /*
   * private void configureDriveMotorsAndSensors() {
   * m_leftLeaderSpark.addFollower(m_leftFollower);
   * m_rightLeaderSpark.addFollower(m_rightFollower);
   *
   * // We need to invert one side of the drivetrain so that positive voltages
   * // result in both sides moving forward. Depending on how your robot's
   * // gearbox is constructed, you might have to invert the left side instead.
   * m_rightLeader.setInverted(true);
   *
   *
   * }
   */

  @Override
  public void resetOdometry(Pose2d pose) {
    super.resetOdometry(pose);

    // Update the pose information in the simulator.
    m_drivetrainSim.setPose(pose);
  }

  @Override
  protected void setMotorVoltages_HAL(double leftVoltage, double rightVoltage) {
    m_leftLeaderSparkSim.setBusVoltage(leftVoltage);
    m_rightLeaderSparkSim.setBusVoltage(rightVoltage);
  }

  @Override
  protected void setSpeeds_HAL(double leftSpeed, double rightSpeed) {
    m_leftLeaderSparkSim.setAppliedOutput(leftSpeed);
    m_rightLeaderSparkSim.setAppliedOutput(rightSpeed);
  }

  @Override
  protected void setSpeeds_HAL(DifferentialDriveWheelSpeeds speeds) {
    double leftPercent = speeds.leftMetersPerSecond;
    double rightPercent = speeds.rightMetersPerSecond;
    setSpeeds(leftPercent, rightPercent);
  }

  @Override
  protected TrivialEncoder getLeftEncoder_HAL() {
    return m_leftTrivialEncoder;
  }

  @Override
  protected TrivialEncoder getRightEncoder_HAL() {
    return m_rightTrivialEncoder;
  }

  @Override
  protected IGyro getGyro_HAL() {
    return m_wrappedGyro;
  }

  @Override
  public double getLeftDistanceMeters() {
    return m_leftEncoder.getPosition();
  }

  @Override
  public double getRightDistanceMeters() {
    return m_rightEncoder.getPosition();
  }

  @Override
  public void periodic() {
    super.periodic();

    // Update the position for the robot that is shown in the simulated field, using
    // the odometry data that will have been computed by the base class.
    var pose = getOdometry().getPoseMeters();
    m_fieldSim.setRobotPose(pose);
    m_fieldSim.getObject("Estimated pose").setPose((getEstimatedPose()));
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSim.setInputs(
        m_leftLeaderSparkSim.getAppliedOutput() * RobotController.getInputVoltage(),
        m_rightLeaderSparkSim.getAppliedOutput() * RobotController.getInputVoltage());

    // Simulated clock ticks forward
    m_drivetrainSim.update(0.02);

    m_leftLeaderSparkSim.iterate(m_drivetrainSim.getLeftVelocityMetersPerSecond()
            / (Math.PI * kWheelRadius.in(Meters)) / 6.06,
        12, 0.02);
    m_rightLeaderSparkSim.iterate(m_drivetrainSim.getRightVelocityMetersPerSecond()
            / (Math.PI * kWheelRadius.in(Meters)) / 6.06,
        12, 0.02);

    // Update the encoders and gyro, based on what the drive train simulation says
    // happend.

    m_leftEncoder.setPosition(m_drivetrainSim.getLeftPositionMeters());
    m_rightEncoder.setPosition(m_drivetrainSim.getRightPositionMeters());
    m_gyroSim.setAngle(-m_drivetrainSim.getHeading().getDegrees());
  }
}
