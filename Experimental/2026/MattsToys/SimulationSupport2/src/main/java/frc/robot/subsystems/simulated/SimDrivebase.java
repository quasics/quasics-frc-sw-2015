// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.games.ReefscapeConstants;
import frc.robot.subsystems.Drivebase;

/**
 * Extension of the Drivebase subsystem for simulation support.
 *
 * Note that this functionality could simply be merged into the Drivebase class
 * (and initially *was* implemented this way), but I'm keeping it separate (for
 * now) to isolate simulation-specific code.
 */
public class SimDrivebase extends Drivebase {
  /**
   * Supported (pre-defined) starting positions for the robot.
   *
   * Note that game-specific starting positions are pre-fixed with the game name
   * (e.g., "Reefscape__").
   */
  public enum StartingPosition {
    /** Default robot position (0, 0, 0). */
    Default,
    /** Facing Blue and aligned with starting game element 1 in Reefscape. */
    Reefscape__Blue1,
    /** Facing Blue and aligned with starting game element 2 in Reefscape. */
    Reefscape__Blue2,
    /** Facing Blue and aligned with starting game element 3 in Reefscape. */
    Reefscape__Blue3,
    /** Facing Red and aligned with starting game element 1 in Reefscape. */
    Reefscape__Red1,
    /** Facing Red and aligned with starting game element 2 in Reefscape. */
    Reefscape__Red2,
    /** Facing Red and aligned with starting game element 3 in Reefscape. */
    Reefscape__Red3,
    Reefscape__Extra1,
    Reefscape__Extra2,
    ;

    /** Returns the robot pose associated with this starting point. */
    public Pose2d getPose() {
      return switch (this) {
        case Default -> DEFAULT_STARTING_POSE;

        case Reefscape__Blue1 ->
          new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
              ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters),
              new Rotation2d(ReefscapeConstants.FACING_BLUE));
        case Reefscape__Blue2 ->
          new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
              ReefscapeConstants.MIDDLE_BALL_HEIGHT.in(Meters),
              new Rotation2d(ReefscapeConstants.FACING_BLUE));
        case Reefscape__Blue3 ->
          new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters),
              ReefscapeConstants.BOTTOM_BALL_HEIGHT.in(Meters),
              new Rotation2d(ReefscapeConstants.FACING_BLUE));
        case Reefscape__Red1 ->
          new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
              ReefscapeConstants.BOTTOM_BALL_HEIGHT.in(Meters),
              new Rotation2d(ReefscapeConstants.FACING_RED));
        case Reefscape__Red2 ->
          new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
              ReefscapeConstants.MIDDLE_BALL_HEIGHT.in(Meters),
              new Rotation2d(ReefscapeConstants.FACING_RED));
        case Reefscape__Red3 ->
          new Pose2d(ReefscapeConstants.RED_STARTING_LINE.in(Meters),
              ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters),
              new Rotation2d(ReefscapeConstants.FACING_RED));

        case Reefscape__Extra1 ->
          new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters) - .5,
              ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters) - .5,
              new Rotation2d(ReefscapeConstants.FACING_BLUE.minus(Degrees.of(5))));
        case Reefscape__Extra2 ->
          new Pose2d(ReefscapeConstants.BLUE_STARTING_LINE.in(Meters) + .5,
              ReefscapeConstants.TOP_BALL_HEIGHT.in(Meters) + .5,
              new Rotation2d(ReefscapeConstants.FACING_BLUE.plus(Degrees.of(5))));
      };
    }

    /**
     * Returns the user-facing name of this starting position (e.g., for use in
     * a chooser).
     */
    public String getName() {
      return this.toString().replaceFirst(".*__", "");
    }
  }

  /** Controls the base class's left encoder under simulation. */
  final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);

  /** Controls the base class's right encoder under simulation. */
  final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  /** Controls the base class's gyro under simulation. */
  final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_rawGyro);

  /**
   * Linear system describing the drive train.
   *
   * Notice that this data will (had better!) look *remarkably* similar to the
   * computed "feed forward" values for this simulated drive base when it is
   * profiled, since they are... well, the actual/ideal values defining that.
   *
   * @see frc.robot.utils.RobotConfigs.DriveFeedForwardConfig
   */
  final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
      // Linear components (velocity, acceleration)
      1.98, 0.2,
      // Angular components (velocity, acceleration)
      1.5, 0.3);

  /** Simulation driver for the overall drive train. */
  final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem,
      // Drive motor type and count
      DCMotor.getNEO(4), GEAR_RATIO,
      TRACK_WIDTH.in(Meters),
      WHEEL_DIAMETER.in(Meters),
      // configure for no noise in measurements
      null);

  /** Constructor. */
  public SimDrivebase() {
    super();

    SendableChooser<StartingPosition> positionChooser = new SendableChooser<StartingPosition>();
    for (var pos : StartingPosition.values()) {
      if (pos == StartingPosition.Default) {
        positionChooser.setDefaultOption(pos.getName(), pos);
      } else {
        positionChooser.addOption(pos.getName(), pos);
      }
    }
    SmartDashboard.putData("Starting point", positionChooser);
    positionChooser.onChange(this::updateStartingPoint);
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

    m_odometry = new DifferentialDriveOdometry(facing, m_leftEncoderSim.getDistance(),
        m_rightEncoderSim.getDistance(), pose);
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
    SimulationUxSupport.instance.updateEstimatedRobotPose("Odometry",
        getEstimatedPose());

    SmartDashboard.putString("Robot pos",
        m_drivetrainSimulator.getPose().getX() + ", " +
            m_drivetrainSimulator.getPose().getY());
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
