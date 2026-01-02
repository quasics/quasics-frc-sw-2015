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
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivebase;

/**
 * Extension of the Drivebase subsystem for simulation support.
 *
 * Note that this functionality could simply be merged into the Drivebase class (and initially *was*
 * implemented this way), but I'm keeping it separate (for now) to isolate simulation-specific code.
 */
public class SimDrivebase extends Drivebase {
  public enum StartingPoint {
    Default,
    Blue1,
    Red1;

    public Pose2d getPose() {
      return switch (this) {
        case Default -> new Pose2d(0, 0, new Rotation2d());
        case Blue1 -> new Pose2d(2, 6, new Rotation2d(Degrees.of(180)));
        case Red1 -> new Pose2d(15.25, 2, new Rotation2d(Degrees.of(0)));
      };
    }
  }

  final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_rawGyro);

  /**
   * Linear system describing the drive train.
   *
   * Notice that this data will (had better!) look *remarkably* similar to the
   * computed "feed forward" values for this simulated drive base, since they
   * are... well, the actual/ideal values defining that.
   *
   * @see frc.robot.utils.RobotConfigs.DriveFeedForwardConfig
   */
  final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);

  /** Simulation driver for the overall drive train. */
  final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(m_drivetrainSystem,
          // Drive motor type and count
          DCMotor.getNEO(4), GEAR_RATIO, TRACK_WIDTH.in(Meters), WHEEL_DIAMETER.in(Meters),
          // configure for no noise in measurements
          null);

  /** Constructor. */
  public SimDrivebase() {
    super();

    SendableChooser<StartingPoint> positionChooser = new SendableChooser<StartingPoint>();
    for (var pos : StartingPoint.values()) {
      if (pos == StartingPoint.Default) {
        positionChooser.setDefaultOption(pos.toString(), pos);
      } else {
        positionChooser.addOption(pos.toString(), pos);
      }
    }
    SmartDashboard.putData("Starting point", positionChooser);
    positionChooser.onChange(this::updateStartingPoint);
  }

  private void updateStartingPoint(StartingPoint position) {
    // Update position data
    Pose2d pose = position.getPose();
    // m_leftEncoderSim.setDistance(0);
    // m_rightEncoderSim.setDistance(0);
    m_gyroSim.setAngle(pose.getRotation().getDegrees());
    m_drivetrainSimulator.setPose(pose);

    m_odometry = new DifferentialDriveOdometry(
        pose.getRotation(), m_leftEncoderSim.getDistance(), m_rightEncoderSim.getDistance(), pose);
  }

  @Override
  public void periodic() {
    super.periodic();

    // Update the field simulation
    SimulationUxSupport.instance.updateFieldRobotPose(m_drivetrainSimulator.getPose());
    SimulationUxSupport.instance.updateEstimatedRobotPose("Odometry", getEstimatedPose());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(m_leftController.get() * RoboRioSim.getVInVoltage(),
        m_rightController.get() * RoboRioSim.getVInVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(0.02);

    // Update the simulated encoders and gyro
    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }
}
