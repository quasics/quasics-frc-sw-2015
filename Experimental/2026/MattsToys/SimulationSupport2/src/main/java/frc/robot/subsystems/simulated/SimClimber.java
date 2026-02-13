package frc.robot.subsystems.simulated;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.util.RobotConfigs.ClimberConfig;

/**
 * Simulated climber subsystem.
 * 
 * This simulates a simple linear actuator with built-in PID control on the
 * motor controller, and optional limit switches at the top and bottom. The
 * climber can be commanded to move to specific positions under PID control, or
 * to extend/retract manually (without PID control). The simulated display will
 * show the current height of the climber, the target position (if any), and the
 * current state (Idle, Manual, At Setpoint, Not At Setpoint).
 * 
 * Note that this is a very basic simulation, and doesn't attempt to model
 * things like motor current limits, stalling, or the effects of gravity. In
 * addition, we're not even using MotorController objects in this; we're just
 * calculating values, based on speeds and time-slicing.
 */
public class SimClimber extends SubsystemBase implements IClimber {
  /**
   * If true, simulate limit switches that stop the climber from moving beyond its
   * physical limits.
   */
  private static final boolean SIMULATE_LIMIT_SWITCHES = true;

  /** Acceptable error for PID control. */
  private static final double ACCEPTABLE_ERROR = 0.05;

  /** Amount to move the climber per simulation cycle when in manual mode. */
  private static final double MANUAL_DELTA_PER_CYCLE = 0.1;

  //
  // Exposed for simulation purposes
  //

  /** Maximum height of the climber (in meters). */
  public static final double MAX_HEIGHT = 1.6;

  /** Minimum height of the climber (in meters). */
  public static final double MIN_HEIGHT = 0.0;

  /** Current target position (when under PID control) */
  Position m_targetPosition = Position.DontCare;

  /** Current state of the climber. */
  State m_state = State.Idle;

  /** Current height of the climber (in meters). */
  double m_currentHeight = MIN_HEIGHT;

  /** PID controller for automatic positioning. */
  private PIDController m_pidController = new PIDController(1, 0, 0);

  /**
   * Constructor.
   * 
   * @param config the climber configuration (used to set PID constants, etc.)
   */
  public SimClimber(ClimberConfig config) {
    setName(SUBSYSTEM_NAME);
    m_pidController = new PIDController(config.pid().kP(), config.pid().kI(), config.pid().kD());
    // Note that I'm ignoring the feedforward term here, since we're simulating the
    // built-in PID on the motor controller, which doesn't have a feedforward
    // component.
  }

  /**
   * Returns the height (in meters) for a given position.
   * 
   * Note that this is exposed for simulation purposes, so that we can easily
   * convert between the enum values and their corresponding heights.
   * 
   * @param pos the position to get the height for
   * @return the height (in meters) for the given position
   */
  public static double getHeightForPosition(Position pos) {
    return switch (pos) {
      case Retracted -> 0.0;
      case Extended -> 1.5;
      case DontCare -> 0.0;
      case PulledUp -> 1.3;
    };
  }

  /** Update the simulated display elements. */
  private void updateSimulatedDisplay() {
    SimulationUxSupport.DeviceStatus status = switch (m_state) {
      case Idle -> SimulationUxSupport.DeviceStatus.Idle;
      case Rising, Descending -> SimulationUxSupport.DeviceStatus.Manual;
      case PidControlled ->
        (Math.abs(m_currentHeight - getHeightForPosition(m_targetPosition)) < ACCEPTABLE_ERROR)
            ? SimulationUxSupport.DeviceStatus.AtSetpoint
            : SimulationUxSupport.DeviceStatus.NotAtSetpoint;
    };

    // Update any simulated display elements here, if needed
    SimulationUxSupport.instance.updateClimber(
        m_currentHeight, getHeightForPosition(m_targetPosition), status);
  }

  @Override
  public void simulationPeriodic() {
    // Simulate behavior of motors during a command loop cycle
    switch (m_state) {
      case PidControlled:
        // Simple PID control to move to target position. (Simulating built-in PID on
        // motor controller; we'd do this in periodic(), otherwise.)
        double targetHeight = getHeightForPosition(m_targetPosition);
        double output = m_pidController.calculate(m_currentHeight, targetHeight);
        m_currentHeight += output * 0.02; // Simulate movement over the 20ms interval
        if (Math.abs(m_currentHeight - targetHeight) < ACCEPTABLE_ERROR) {
          m_currentHeight = targetHeight;
          m_state = State.Idle;
        }
        break;
      case Rising:
        m_currentHeight = Math.max(m_currentHeight + MANUAL_DELTA_PER_CYCLE, MAX_HEIGHT);
        if (m_currentHeight >= MAX_HEIGHT) {
          if (SIMULATE_LIMIT_SWITCHES) {
            m_state = State.Idle;
          }
        }
        break;
      case Descending:
        m_currentHeight = Math.max(m_currentHeight + MANUAL_DELTA_PER_CYCLE, MAX_HEIGHT);
        if (m_currentHeight >= MAX_HEIGHT) {
          if (SIMULATE_LIMIT_SWITCHES) {
            m_state = State.Idle;
          }
        }
        break;
      case Idle:
        // Do nothing....
        break;
    }

    updateSimulatedDisplay();
  }

  @Override
  public State getCurrentState() {
    return m_state;
  }

  @Override
  public void moveToPosition(Position position) {
    if (position == Position.DontCare) {
      // Don't have a defined position to move to; ignore the command
      stop();
    } else {
      // Move to the specified position under PID control
      m_state = State.PidControlled;
      m_targetPosition = position;
    }
  }

  @Override
  public void extend() {
    m_state = State.Rising;
  }

  @Override
  public void retract() {
    m_state = State.Descending;
  }

  @Override
  public void stop() {
    m_state = State.Idle;
    m_targetPosition = Position.DontCare;
  }

  @Override
  public void close() throws IOException {
    // No-op
  }
}
