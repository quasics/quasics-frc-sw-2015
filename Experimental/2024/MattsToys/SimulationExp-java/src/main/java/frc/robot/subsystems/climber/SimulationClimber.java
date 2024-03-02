package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SimulationPorts;

/**
 * Initial/simple simulation of the climber subsystem functionality, using the
 * WPILib's built-in support for modeling elevators to present a view of the
 * left/right arms. This will also provide a rendering of the simulated arm
 * positions via a Mechanism2d view on the SmartDashboard interface.
 */
public class SimulationClimber extends AbstractClimber {
  static final double EXTENSION_SPEED = +1.0;
  static final double RETRACTION_SPEED = -1.0;
  static final double LEFT_SCALING_FACTOR = 1.0;
  static final double RIGHT_SCALING_FACTOR = 0.9; // Simulate right side being slower

  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_leftGearing = DCMotor.getNEO(1);
  private final DCMotor m_rightGearing = DCMotor.getNEO(1);

  private final Encoder m_leftEncoder = new Encoder(SimulationPorts.LEFT_CLIMBER_ENCODER_PORT_A,
      SimulationPorts.LEFT_CLIMBER_ENCODER_PORT_B);
  private final Encoder m_rightEncoder = new Encoder(SimulationPorts.RIGHT_CLIMBER_ENCODER_PORT_A,
      SimulationPorts.RIGHT_CLIMBER_ENCODER_PORT_B);
  private final PWMSparkMax m_leftMotor = new PWMSparkMax(SimulationPorts.LEFT_CLIMBER_PWM_ID);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(SimulationPorts.RIGHT_CLIMBER_PWM_ID);

  // Mechanism2d visualization of the climber arms (for rendering in
  // SmartDashboard, or the simulator).
  private final MechanismLigament2d m_leftClimberMech2d;
  private final MechanismLigament2d m_rightClimberMech2d;

  //////////////////////////////////////////////////////////////////////////////
  // Simulation support data/objects

  // TODO: Update these constants to better emulate the real behavior of the
  // climbers. (But for now, this will at least give us something we can use.)
  private static final double kClimberGearing = 10.0;
  private static final double kClimberDrumRadius = Units.inchesToMeters(1);
  private static final double kClimberEncoderDistPerPulse = 2.0 * Math.PI * kClimberDrumRadius / 4096;
  private static final double kCarriageMass = 1.0; // kg
  private static final double kMinClimberHeightMeters = -1.0; // arbitrary: should be < min desired
  private static final double kMaxClimberHeightMeters = 8; // arbitrary: should be > max desired
  private static final boolean ENABLE_GRAVITY = true;
  private static final double kClimberHeightMetersAtStart = 0;

  // Simulation classes help us simulate what's going on, optionally including
  // gravity. We're going to model the climber arms as elevators, since that's
  // a reasonably close mechanism (and it's predefined in WPILib).
  private final ElevatorSim m_leftClimberSim = new ElevatorSim(
      m_leftGearing,
      kClimberGearing,
      kCarriageMass,
      kClimberDrumRadius,
      kMinClimberHeightMeters,
      kMaxClimberHeightMeters,
      ENABLE_GRAVITY,
      kClimberHeightMetersAtStart);
  private final ElevatorSim m_rightClimberSim = new ElevatorSim(
      m_rightGearing,
      kClimberGearing,
      kCarriageMass,
      kClimberDrumRadius,
      kMinClimberHeightMeters,
      kMaxClimberHeightMeters,
      ENABLE_GRAVITY,
      kClimberHeightMetersAtStart);

  // Simulation motors/encoders
  private final PWMSim m_leftMotorSim = new PWMSim(m_leftMotor);
  private final PWMSim m_rightMotorSim = new PWMSim(m_rightMotor);
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  public SimulationClimber() {
    // Encoder setup (so that simulation can drive actual values; without this,
    // we'll keep getting 0 distance/height, regardless of
    // direction/speed/duration).
    m_leftEncoder.setDistancePerPulse(kClimberEncoderDistPerPulse);
    m_rightEncoder.setDistancePerPulse(kClimberEncoderDistPerPulse);

    // Simulation rendering setup.
    Mechanism2d m_mech2d = new Mechanism2d(9, 10);
    m_leftClimberMech2d = m_mech2d.getRoot("LeftClimber Root", 3, 0).append(
        new MechanismLigament2d("LeftClimber", m_leftClimberSim.getPositionMeters(), 90));
    m_rightClimberMech2d = m_mech2d.getRoot("RightClimber Root", 6, 0).append(
        new MechanismLigament2d("RightClimber", m_rightClimberSim.getPositionMeters(), 90));

    // Publish Mechanism2d to SmartDashboard.
    // To view the Climber visualization, select Network Tables -> SmartDashboard
    // -> Climber Sim
    SmartDashboard.putData("Climber Sim", m_mech2d);
  }

  public void periodic() {
    super.periodic();

    // Update the visualization of the climber positions.
    //
    // This might be done in simulationPeriodic(), since this class *is* purely
    // simulation-oriented. But I'll do it in the periodic() function, as a reminder
    // that this same thing could be done to provide a rendering of the data for a
    // *real* elevator within the SmartDashboard at a match (e.g., as an aid to the
    // drive team).
    m_leftClimberMech2d.setLength(m_leftEncoder.getDistance());
    m_rightClimberMech2d.setLength(m_rightEncoder.getDistance());
  }

  /** Advance the simulation. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our subsystem is doing.

    // First, we set our "inputs" (voltages).
    m_leftClimberSim.setInput(m_leftMotorSim.getSpeed() * RobotController.getBatteryVoltage());
    m_rightClimberSim.setInput(m_rightMotorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update the simulation. The standard loop time is 20ms.
    m_leftClimberSim.update(0.020);
    m_rightClimberSim.update(0.020);

    // Now, we can set our simulated encoder's readings and simulated battery
    // voltage.
    final double leftPos = m_leftClimberSim.getPositionMeters();
    final double rightPos = m_rightClimberSim.getPositionMeters();
    m_leftEncoderSim.setDistance(leftPos);
    m_rightEncoderSim.setDistance(rightPos);
    RoboRioSim.setVInVoltage(
        // Note: this should really be updated in conjunction with the simulated drive
        // base (and anything else we're "powering", such as a simulated shooter, as
        // well).
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_leftClimberSim.getCurrentDrawAmps(), m_rightClimberSim.getCurrentDrawAmps()));
  }

  @Override
  protected void resetLeftEncoder_HAL() {
    // Note that this is (as yet) untested.
    m_leftEncoderSim.setDistance(0);
  }

  @Override
  protected void resetRightEncoder_HAL() {
    // Note that this is (as yet) untested.
    m_rightEncoderSim.setDistance(0);
  }

  @Override
  protected double getLeftRevolutions_HAL() {
    return m_leftEncoder.getDistance();
  }

  @Override
  protected double getRightRevolutions_HAL() {
    return m_rightEncoder.getDistance();
  }

  @Override
  protected void stopLeftClimber_HAL() {
    m_leftMotor.set(0);
  }

  @Override
  protected void stopRightClimber_HAL() {
    m_rightMotor.set(0);
  }

  @Override
  protected void extendLeftClimber_HAL() {
    m_leftMotor.set(EXTENSION_SPEED * LEFT_SCALING_FACTOR);
  }

  @Override
  protected void extendRightClimber_HAL() {
    m_rightMotor.set(EXTENSION_SPEED * RIGHT_SCALING_FACTOR);
  }

  @Override
  protected void retractLeftClimber_HAL() {
    m_leftMotor.set(RETRACTION_SPEED * LEFT_SCALING_FACTOR);
  }

  @Override
  protected void retractRightClimber_HAL() {
    m_rightMotor.set(RETRACTION_SPEED * RIGHT_SCALING_FACTOR);
  }
}
