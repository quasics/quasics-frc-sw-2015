package frc.robot.subsystems.simulated;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.util.RobotConfigs.ClimberConfig;

public class SimClimber extends SubsystemBase implements IClimber {
  private static final boolean SIMULATE_LIMIT_SWITCHES = true;
  private static final double ACCEPTABLE_ERROR = 0.05;
  private static final double MANUAL_DELTA_PER_CYCLE = 0.1;

  //
  // Exposed for simulation purposes
  //

  public static final double MAX_HEIGHT = 1.6;
  public static final double MIN_HEIGHT = 0.0;

  Position m_targetPosition = Position.DontCare;
  State m_state = State.Idle;
  double m_currentHeight = MIN_HEIGHT;

  /** PID controller for automatic positioning. */
  private PIDController m_pidController = new PIDController(1, 0, 0);

  public SimClimber(ClimberConfig config) {
    setName(SUBSYSTEM_NAME);
    m_pidController = new PIDController(config.pid().kP(), config.pid().kI(), config.pid().kD());
    // Note that I'm ignoring the feedforward term here, since we're simulating the
    // built-in PID on the motor controller, which doesn't have a feedforward
    // component.
  }

  // Exposed for simulation purposes
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
    m_state = State.PidControlled;
    m_targetPosition = position;
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
  }

  @Override
  public void close() throws IOException {
    // No-op
  }
}
