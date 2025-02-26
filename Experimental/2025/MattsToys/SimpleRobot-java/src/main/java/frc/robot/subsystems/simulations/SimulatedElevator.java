// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.AbstractElevator;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Completes the AbstractElevator class definition for the purpose of
 * controlling simulated hardware.
 */
public class SimulatedElevator extends AbstractElevator {
  /** Extension speed while running under manual control. */
  static final double EXTENSION_SPEED = +1.0;
  /** Retraction speed while running under manual control. */
  static final double RETRACTION_SPEED = -1.0;

  static final Distance MAX_SAFE_SPAN = MAX_SAFE_HEIGHT.minus(MIN_SAFE_HEIGHT);

  // TODO: Update these constants to better emulate the real behavior of the
  // hardware. (But for now, this will at least give us something we can use.)
  private static final double kGearing = 20.0; // Arbitrary (but needs to be enough for the simulated physics to work)
  private static final Distance kDrumRadius = Inches.of(1);
  private static final double kEncoderMetersPerPulse = 2.0 * Math.PI * kDrumRadius.abs(Meters) / 4096;
  private static final double kCarriageMass = 1.0; // kg
  private static final Distance kMinSimulationHeight = MIN_SAFE_HEIGHT.minus(Meters.of(-0.1)); // arbitrary: should be
                                                                                               // <= min desired
  private static final Distance kMaxSimulationHeight = MAX_SAFE_HEIGHT.plus(Meters.of(0.25)); // arbitrary: should be >=
                                                                                              // max desired
  private static final Distance kStartingSimulationHeight = MIN_SAFE_HEIGHT;
  private static final boolean ENABLE_GRAVITY = true;

  /** This gearbox represents a gearbox containing 2 NEO motors. */
  private final DCMotor m_gearing = DCMotor.getNEO(2);

  /** Motor controller driving the elevator. */
  private final SparkMax m_motor = new SparkMax(SimulationPorts.ELEVATOR_CAN_ID, MotorType.kBrushless);

  /** Encoder tracking the current position of the elevator. */
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  // Note: arbitrary values; we'd want to define something real.
  private final PIDController m_pid;
  private final ElevatorFeedforward m_feedforward;

  /**
   * Mechanism2d visualization of the hardware (for rendering in SmartDashboard,
   * or the simulator).
   */
  private final MechanismLigament2d m_mech2d;

  /** Color used to render the elevator when running under manual control. */
  private final static Color8Bit NO_SETPOINT = new Color8Bit(255, 165, 0);

  /**
   * Color used to render the elevator when we've reached the target position .
   */
  private final static Color8Bit AT_SETPOINT = new Color8Bit(0, 255, 0);

  /**
   * Color used to render the elevator when we're driving towards the target
   * position .
   */
  private final static Color8Bit NOT_AT_SETPOINT = new Color8Bit(255, 0, 0);

  //////////////////////////////////////////////////////////////////////////////
  // Simulation support data/objects

  /** Motor being driven by the controller. */
  private DCMotor elevatorPlant = DCMotor.getNEO(1);

  // Simulation motors/encoders
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, elevatorPlant);

  // Physics simulation control.
  private final ElevatorSim m_sim = new ElevatorSim(m_gearing, kGearing, kCarriageMass,
      kDrumRadius.in(Meters), kMinSimulationHeight.in(Meters), kMaxSimulationHeight.in(Meters),
      ENABLE_GRAVITY, kStartingSimulationHeight.in(Meters));

  /**
   * Creates a new SimulatedElevator.
   *
   * @param robotConfig configuration of the robot being targeted
   */
  public SimulatedElevator(RobotConfig robotConfig) {
    var pidConfig = robotConfig.elevator().pid();
    var ffConfig = robotConfig.elevator().feedForward();

    m_pid = new PIDController(pidConfig.kP(), pidConfig.kI(), pidConfig.kD());
    m_feedforward = new ElevatorFeedforward(
        ffConfig.kS().in(Volts), ffConfig.kG().in(Volts), ffConfig.kV(), ffConfig.kA());

    // Configure the motor.
    var motorConfig = new SparkMaxConfig();
    motorConfig.encoder.positionConversionFactor(kEncoderMetersPerPulse);
    motorConfig.encoder.velocityConversionFactor(kEncoderMetersPerPulse / 60);
    m_motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pid.setTolerance(0.02); // within 2cm is fine

    m_motorSim.setPosition(0);

    // Simulation rendering setup.
    Mechanism2d rootMech2d = new Mechanism2d(9, MAX_SAFE_HEIGHT.in(Meters) * 1.15 /* Leave a little room at the top */);
    m_mech2d = rootMech2d.getRoot("LeftClimber Root", 3, 0)
        .append(new MechanismLigament2d("LeftClimber", m_sim.getPositionMeters(), 90));

    // Publish Mechanism2d to SmartDashboard.
    // To show the visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", rootMech2d);
  }

  @Override
  protected void updateMotor_impl() {
    final boolean noisy = true;

    final Distance setpoint = getPositionForTarget(m_target);
    final double velocity = m_encoder.getVelocity();
    final double pidOutput = m_pid.calculate(m_encoder.getPosition(), setpoint.in(Meters));
    final double feedForward = m_feedforward.calculate(m_encoder.getVelocity());

    final double output = MathUtil.clamp(pidOutput + feedForward, -12.0, +12.0);
    m_motor.setVoltage(output);

    if (noisy) {
      System.out.printf("PID -> pos: %.02f, set: %.02f, vel: %.02f, pidOut: %.02f, ff: %.02f, "
          + "output: %.02f, atSetpoint: %b%n",
          m_encoder.getPosition(), setpoint.in(Meters), velocity, pidOutput, feedForward, output,
          m_pid.atSetpoint());
    }
  }

  @Override
  public void periodic() {
    super.periodic();

    // Update the visualization of the climber positions.
    //
    // This might be done in simulationPeriodic(), since this class *is* purely
    // simulation-oriented. But I'll do it in the periodic() function, as a reminder
    // that this same thing could be done to provide a rendering of the data for a
    // *real* elevator within the SmartDashboard at a match (e.g., as an aid to the
    // drive team).
    m_mech2d.setLength(m_encoder.getPosition());
    if (m_target == TargetPosition.DontCare) {
      m_mech2d.setColor(NO_SETPOINT);
    } else if (m_pid.atSetpoint()) {
      m_mech2d.setColor(AT_SETPOINT);
    } else {
      m_mech2d.setColor(NOT_AT_SETPOINT);
    }
  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    final boolean noisy = false;

    super.simulationPeriodic();

    // In this method, we update our simulation of what our subsystem is doing.

    // First we set out "inputs" (voltages).
    final double initialPos = m_encoder.getPosition();
    var appliedOutput = m_motorSim.getAppliedOutput();
    var voltsIn = RoboRioSim.getVInVoltage();
    m_sim.setInput(0, appliedOutput * voltsIn);

    // Next, we update the simulation. The standard loop time is 20ms.
    final double timeIncrement = 0.020;
    m_sim.update(timeIncrement);

    // Per original example, if we don't do this, the rendered value is off a
    // little bit.
    m_motorSim.setPosition(m_sim.getPositionMeters());

    // Update the motor's idea of how it is moving.
    var motorSpeed = m_sim.getVelocityMetersPerSecond();
    m_motorSim.iterate(motorSpeed, voltsIn, timeIncrement);

    if (noisy) {
      System.out.println("Sim --> initial: " + initialPos + ", post: " + m_sim.getPositionMeters()
          + ", speed: " + motorSpeed);
    }

    RoboRioSim.setVInVoltage(
        // Note: this should really be updated in conjunction with the simulated drive
        // base (and anything else we're "powering", such as a simulated shooter, as
        // well).
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps()));
  }

  @Override
  protected void resetEncoder_impl() {
    m_encoder.setPosition(0);
  }

  @Override
  protected Distance getHeight_impl() {
    return Meters.of(m_encoder.getPosition());
  }

  @Override
  protected void stop_impl() {
    m_motor.set(0);
  }

  @Override
  protected void extend_impl() {
    m_motor.set(EXTENSION_SPEED);
  }

  @Override
  protected void retract_impl() {
    m_motor.set(RETRACTION_SPEED);
  }

  /**
   * @param targetPosition the logical position to which the elevator should move
   * @return the height to which the elevator should move
   */
  protected Distance getPositionForTarget(TargetPosition targetPosition) {
    switch (targetPosition) {
      case DontCare:
        // Wherever we are right now is fine, thanks.
        return Meters.of(m_encoder.getPosition());

      case Bottom:
        return MIN_SAFE_HEIGHT;
      case Top:
        return MAX_SAFE_HEIGHT;

      case L1:
        return MIN_SAFE_HEIGHT.plus(MAX_SAFE_SPAN.times(1.0 / 3.0));
      case L2:
        return MIN_SAFE_HEIGHT.plus(MAX_SAFE_SPAN.times(2.0 / 3.0));
    }

    System.err.println("Unrecognized target position: " + targetPosition);
    return MIN_SAFE_HEIGHT;
  }

  @Override
  public void setTargetPosition(TargetPosition targetPosition) {
    if (targetPosition != m_target) {
      m_pid.reset();
    }

    super.setTargetPosition(targetPosition);
  }

  @Override
  public boolean atTargetPosition() {
    if (m_target == TargetPosition.DontCare) {
      return true;
    } else {
      return m_pid.atSetpoint();
    }
  }
}
