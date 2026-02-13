// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.games.Game;
import frc.robot.constants.games.StartingPosition;
import frc.robot.constants.robots.SimulationPorts;
import frc.robot.hardware.actuators.IMotorControllerPlus;
import frc.robot.hardware.sensors.IGyro;
import frc.robot.hardware.sensors.TrivialEncoder;
import frc.robot.subsystems.DrivebaseBase;
import frc.robot.util.config.DriveConfig;

/**
 * Extension of the Drivebase subsystem for simulation support.
 *
 * Note that this functionality could simply be merged into the Drivebase class
 * (and initially *was* implemented this way), but I'm keeping it separate (for
 * now) to isolate simulation-specific code.
 */
public class SimDrivebase extends DrivebaseBase {
  final static boolean USE_MASS_AND_MOI_FOR_SIM_SETUP = false;

  /** Controls the base class's left encoder under simulation. */
  final EncoderSim m_leftEncoderSim;

  /** Controls the base class's right encoder under simulation. */
  final EncoderSim m_rightEncoderSim;

  /** Controls the base class's gyro under simulation. */
  final AnalogGyroSim m_gyroSim;

  /** Mass of the simulated robot. */
  final static Mass SIMULATED_ROBOT_MASS = Kilograms.of(55.0); // ~121 pounds

  /** Moment of intertia for the simulated robot, in kg * m^2 (typically 3-8). */
  final static double MOMENT_OF_INERTIA = 7.5;

  /** Simulation driver for the overall drive train. */
  final DifferentialDrivetrainSim m_drivetrainSimulator = (USE_MASS_AND_MOI_FOR_SIM_SETUP
      ? new DifferentialDrivetrainSim(
          // Drive motor type and count (per side)
          DCMotor.getNEO(2),
          // Gear ratio
          GEAR_RATIO,
          // Moment of intertia (joules/(kg*m^2))
          MOMENT_OF_INERTIA,
          // Robot mass (kg)
          SIMULATED_ROBOT_MASS.in(Kilograms),
          // Wheel radius (m)
          WHEEL_DIAMETER.in(Meters) / 2,
          // Track width (m)
          TRACK_WIDTH.in(Meters),
          // configure for no noise in measurements
          null)
      : new DifferentialDrivetrainSim(
          // Linear system describing the drive train.
          //
          // Notice that this data will (had better!) look *remarkably* similar to the
          // computed "feed forward" values for this simulated drive base when it is
          // profiled, since they are... well, the actual/ideal values defining that.
          LinearSystemId.identifyDrivetrainSystem(
              // Linear components (velocity, acceleration)
              1.98, 0.2,
              // Angular components (velocity, acceleration)
              1.5, 0.3),
          // Drive motor type and count (per side)
          DCMotor.getNEO(2),
          // Gear ratio
          GEAR_RATIO,
          // Track width (m)
          TRACK_WIDTH.in(Meters),
          // Wheel radius (m)
          WHEEL_DIAMETER.in(Meters) / 2,
          // configure for no noise in measurements
          null));

  /**
   * Constructor.
   *
   * @param config the configuration for this drive base
   */
  public SimDrivebase(DriveConfig config) {
    this(config,
        // Left encoder
        getSimulatedEncoderPair(SimulationPorts.DIO.LEFT_ENCODER_A_PORT,
            SimulationPorts.DIO.LEFT_ENCODER_B_PORT,
            config.orientation().isLeftInverted()),
        // Right encoder
        getSimulatedEncoderPair(SimulationPorts.DIO.RIGHT_ENCODER_A_PORT,
            SimulationPorts.DIO.RIGHT_ENCODER_B_PORT,
            config.orientation().isRightInverted()),
        // Gyro
        new AnalogGyro(SimulationPorts.Channel.GYRO_PORT));
  }

  /**
   * Actual constructor for this class.
   *
   * @param config  drivebase configuration data
   * @param left    left encoder/simulator pair
   * @param right   right encoder/simulator pair
   * @param rawGyro gyro
   */
  protected SimDrivebase(DriveConfig config, SimulatedEncoderPair left,
      SimulatedEncoderPair right, AnalogGyro rawGyro) {
    super(config,
        IMotorControllerPlus.forPWMMotorController(
            new PWMSparkMax(SimulationPorts.PWM.LEFT_MOTOR_PORT)),
        IMotorControllerPlus.forPWMMotorController(
            new PWMSparkMax(SimulationPorts.PWM.RIGHT_MOTOR_PORT)),
        TrivialEncoder.forWpiLibEncoder(left.encoder, left.encoderSim),
        TrivialEncoder.forWpiLibEncoder(right.encoder, right.encoderSim),
        IGyro.wrapGyro(rawGyro), true);

    m_leftEncoderSim = left.encoderSim;
    m_rightEncoderSim = right.encoderSim;
    m_gyroSim = new AnalogGyroSim(rawGyro);

    updateStartingPointSelector(null);
  }

  /** Used to hold a WPI encoder and its paired EncoderSim object. */
  private record SimulatedEncoderPair(Encoder encoder, EncoderSim encoderSim) {
  }

  /**
   * Allocates an encoder and a paired simulator object.
   *
   * @param portId1  "A" port for the encoder
   * @param portId2  "B" port for the encoder
   * @param inverted if true, the motor is in an inverted configuration
   * @return the simulated encoder pair
   */
  protected static SimulatedEncoderPair getSimulatedEncoderPair(
      int portId1, int portId2, boolean inverted) {
    Encoder e = getConfiguredEncoder(portId1, portId2, inverted);
    EncoderSim sim = new EncoderSim(e);
    return new SimulatedEncoderPair(e, sim);
  }

  /**
   * Configures the "starting point" selector.
   *
   * @param game if non-null, specifies the game for which starting positions
   *             should have the prefix removed from their names when shown to
   *             the user (for faster identification)
   */
  private void updateStartingPointSelector(Game game) {
    SendableChooser<StartingPosition> positionChooser = new SendableChooser<StartingPosition>();
    for (var pos : StartingPosition.values()) {
      String name = pos.getNameWithoutGamePrefix(game);
      if (pos == StartingPosition.Default) {
        positionChooser.setDefaultOption(name, pos);
      } else {
        positionChooser.addOption(name, pos);
      }
    }
    SmartDashboard.putData("Starting point", positionChooser);
    positionChooser.onChange(this::updateStartingPoint);
  }

  public void setGame(Game game) {
    updateStartingPointSelector(game);
  }

  /**
   * Listener method, used to handle changes to the "starting point" chooser.
   *
   * @param position new selected position
   */
  private void updateStartingPoint(StartingPosition position) {
    final Pose2d pose = position.getPose();
    final Rotation2d facing = pose.getRotation();

    m_gyroSim.setAngle(facing.getDegrees());
    m_drivetrainSimulator.setPose(pose);
    m_leftEncoderSim.setDistance(0);
    m_rightEncoderSim.setDistance(0);

    m_odometry = new DifferentialDriveOdometry(facing,
        m_leftEncoderSim.getDistance(), m_rightEncoderSim.getDistance(), pose);
  }

  //
  // SubsystemBase methods
  //

  @Override
  public void periodic() {
    super.periodic();

    // Update the field simulation (based on calculations in
    // simulationPeriodic).
    SimulationUxSupport.instance.updateFieldRobotPose(
        m_drivetrainSimulator.getPose());
    SimulationUxSupport.instance.updateEstimatedRobotPose(
        "Odometry", getEstimatedPose());

    SmartDashboard.putString("Robot pos",
        m_drivetrainSimulator.getPose().getX() + ", "
            + m_drivetrainSimulator.getPose().getY());
  }

  @Override
  public void simulationPeriodic() {
    final double dtSeconds = 0.02;

    // Save off the old heading for angular velocity computation.
    final var oldHeading = m_drivetrainSimulator.getHeading();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        m_leftController.get() * RoboRioSim.getVInVoltage(),
        m_rightController.get() * RoboRioSim.getVInVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(dtSeconds);

    // Update the simulated encoders and gyro
    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_rightEncoderSim.setDistance(
        m_drivetrainSimulator.getRightPositionMeters());
    m_leftEncoderSim.setRate(
        m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setRate(
        m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    // Angular velocity is not computed by the drive train simulator, but it's
    // just the derivative of heading (change in heading over time), so we can
    // compute it here.
    final var deltaHeading = m_drivetrainSimulator.getHeading().minus(oldHeading);
    final double angularVelocity = deltaHeading.getDegrees() / dtSeconds; // degrees per second
    m_gyroSim.setRate(angularVelocity);
  }
}
