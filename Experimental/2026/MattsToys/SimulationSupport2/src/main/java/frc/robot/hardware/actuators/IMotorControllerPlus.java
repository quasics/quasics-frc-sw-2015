// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware.actuators;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import java.io.Closeable;
import java.io.IOException;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This interface extends the standard WPILib MotorController type so that it
 * also supports additional functionality (e.g., retrieving the motor voltage
 * for use during characterization, closure, etc.).
 * 
 * Possible future additions to this interface include:
 * <ul>
 * <li>
 * Adding support for PID control, either directly using onboard controllers
 * (when supported by the hardware, such as on a SparkMax) or by implementing it
 * using the PIDController class and a velocity sensor (i.e., encoder). This
 * would be in addition to the existing "set speed" functionality, not a
 * replacement for it.)
 * <p>
 * If we do add PID control support, we should also consider how to handle
 * feedforward control. One option would be to allow users to specify a
 * feedforward function that calculates the necessary voltage based on the
 * target velocity, and then have the PID control loop automatically add that
 * feedforward voltage to the output. This would make it easier to use
 * feedforward control without having to manually calculate and apply the
 * feedforward voltage in the user code.
 * </ul>
 */
public interface IMotorControllerPlus extends MotorController {
  /** @return the voltage currently being applied to the motor. */
  Voltage getVoltage();

  /** Closes the motor. (Duplicates the signature for java.lang.Closeable.) */
  void close() throws IOException;

  /**
   * @return true iff this motor controller supports "braking mode".
   *
   * @see #setBrakeMode(boolean)
   */
  boolean canSetBrakeMode();

  /**
   * Enabled/disables "brake mode" on the motor, if supported by the underlying
   * hardware.
   *
   * @param enabled if true, sets brake mode on the motor
   */
  void setBrakeMode(boolean enabled);

  /**
   * Wraps the provided WPILib controller with this interface.
   *
   * @param controller the ("raw") motor controller being wrapped
   * @return a wrapped controller, implementing IMotorControllerPlus
   */
  static IMotorControllerPlus forPWMMotorController(
      PWMMotorController controller) {
    return new MotorControllerPlus(controller,
        () -> Volts.of(controller.getVoltage()),
        () -> controller.close(), false, (Boolean b) -> {
        });
  }

  /**
   * A class implementing the IMotorControllerPlus, passing through base type
   * functionality to an underlying controller, and using functors to provide
   * the implementations of the extra functionality.
   */
  public class MotorControllerPlus implements IMotorControllerPlus {
    /** An underlying (wrapped/"raw") motor controller. */
    final MotorController m_controller;

    /**
     * Supplies the voltage being sent to the motor.
     *
     * @see #getVoltage()
     */
    final Supplier<Voltage> m_voltageSupplier;

    /**
     * Used to close the underlying motor controller.
     *
     * @see #close()
     */
    final Closeable m_closer;

    final boolean m_brakeModeSupported;
    final Consumer<Boolean> m_brakeModeSetter;

    /**
     * Constructor.
     *
     * @param controller      an underlying (wrapped/"raw") motor controller
     * @param voltageSupplier spplies the voltage being sent to the motor
     * @param closer          used to close the underlying motor controller
     */
    public MotorControllerPlus(MotorController controller,
        Supplier<Voltage> voltageSupplier, Closeable closer,
        boolean brakeModeSupported, Consumer<Boolean> brakeModeSetter) {
      m_controller = controller;
      m_voltageSupplier = voltageSupplier;
      m_closer = closer;
      m_brakeModeSetter = brakeModeSetter;
      m_brakeModeSupported = brakeModeSupported;
    }

    //
    // Methods from MotorController
    //

    @Override
    public void set(double speed) {
      m_controller.set(speed);
    }

    @Override
    public double get() {
      return m_controller.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
      m_controller.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
      return m_controller.getInverted();
    }

    @Override
    public void disable() {
      m_controller.disable();
    }

    @Override
    public void stopMotor() {
      m_controller.stopMotor();
    }

    //
    // Methods from IMotorControllerPlus
    //
    @Override
    public Voltage getVoltage() {
      return m_voltageSupplier.get();
    }

    @Override
    public void close() throws IOException {
      m_closer.close();
    }

    @Override
    public boolean canSetBrakeMode() {
      return m_brakeModeSupported;
    }

    @Override
    public void setBrakeMode(boolean enabled) {
      m_brakeModeSetter.accept(enabled);
    }
  }
}
