package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.Drivebase;

/**
 * Extension of the Drivebase subsystem for simulation support.
 *
 * Note that this functionality could simply be merged into the Drivebase class (and initially *was*
 * implemented this way), but I'm keeping it separate (for now) to isolate simulation-specific code.
 */
public class SimDrivebase extends Drivebase {
  final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  final AnalogGyroSim m_gyroSim = new AnalogGyroSim(rawGyro);

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
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(leftController.get() * RoboRioSim.getVInVoltage(),
        rightController.get() * RoboRioSim.getVInVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(0.02);

    // Update the simulated encoders and gyro
    leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

    // Update the field simulation
    SimulationUxSupport.instance.updateFieldRobotPose(m_drivetrainSimulator.getPose());
  }
}
