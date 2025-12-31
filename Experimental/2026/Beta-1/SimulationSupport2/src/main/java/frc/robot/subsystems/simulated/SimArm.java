package frc.robot.subsystems.simulated;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ISingleJointArm;

/**
 *
 * Note: arm angles are measured from the vertical.
 */
public class SimArm extends SubsystemBase implements ISingleJointArm {
  final static double MANUAL_CONTROL_SPEED = 0.05;
  final static double SETPOINT_TOLERANCE = 0.01;

  final public static Angle ARM_MIN = Degrees.of(100);
  final public static Angle ARM_MAX = Degrees.of(-10);
  final public static Angle ARM_OUT = Degrees.of(90);
  final public static Angle ARM_UP = Degrees.of(0);

  private State state = State.IDLE;
  private Angle targetAngle = null;
  private Angle currentAngle = null;

  /** PID controller for automatic positioning. */
  private PIDController pidController = new PIDController(1, 0, 0);

  public SimArm() {
    setName(SUBSYSTEM_NAME);
    currentAngle = Degrees.of(45);
  }

  /** Update the simulated display elements. */
  private void updateSimulatedDisplay() {
    SimulationUxSupport.DeviceStatus status = switch (state) {
      case IDLE -> SimulationUxSupport.DeviceStatus.Idle;
      case MOVING_TO_POSITION ->
        (Math.abs(currentAngle.minus(targetAngle).in(Degrees)) < SETPOINT_TOLERANCE)
            ? SimulationUxSupport.DeviceStatus.AtSetpoint
            : SimulationUxSupport.DeviceStatus.NotAtSetpoint;
    };

    // Update any simulated display elements here, if needed
    SimulationUxSupport.instance.updateArm(currentAngle, status);
  }

  final static boolean USE_PID = true;

  @Override
  public void simulationPeriodic() {
    if (state == State.IDLE) {
      // Nothing to do.
    } else if (state == State.MOVING_TO_POSITION) {
      if (USE_PID) {
        // Simple PID control to move to target angle
        double output = pidController.calculate(currentAngle.in(Degrees), targetAngle.in(Degrees));
        System.out.println("Current: " + currentAngle.in(Degrees)
            + ", target: " + targetAngle.in(Degrees) + ", output: " + output);
        currentAngle = currentAngle.plus(Degrees.of(output * 0.02)); // Simulate movement over 20ms
      } else {
        final double errorDegrees = targetAngle.minus(currentAngle).in(Degrees);
        final double sign = Math.signum(errorDegrees);
        double delta = MANUAL_CONTROL_SPEED * sign;
        if (Math.abs(errorDegrees) < Math.abs(delta)) {
          delta = errorDegrees;
        }
        System.out.println("Current: " + currentAngle.in(Degrees)
            + ", target: " + targetAngle.in(Degrees) + ", delta: " + delta);
        currentAngle = currentAngle.plus(Degrees.of(delta)); // Simulate movement over 20ms
      }
    }

    updateSimulatedDisplay();
  }

  @Override
  public void stop() {
    state = State.IDLE;
    targetAngle = null;
  }

  @Override
  public void setTargetPosition(Angle targetPosition) {
    state = State.MOVING_TO_POSITION;
    targetAngle = targetPosition;
  }

  @Override
  public Angle getCurrentAngle() {
    return currentAngle;
  }

  @Override
  public Angle getArmMinAngle() {
    return Degrees.of(-5);
  }

  @Override
  public Angle getArmMaxAngle() {
    return Degrees.of(110);
  }

  @Override
  public Angle getArmOutAngle() {
    return ARM_OUT;
  }

  @Override
  public Angle getArmUpAngle() {
    return ARM_UP;
  }
}
