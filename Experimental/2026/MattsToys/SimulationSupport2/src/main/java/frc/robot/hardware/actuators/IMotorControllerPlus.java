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
import java.util.function.Supplier;

/**
 * This interface extends the standard WPILib MotorController type so that it
 * also supports retrieving the motor voltage (e.g., for use during
 * characterization) and closure.
 */
public interface IMotorControllerPlus extends MotorController {
  /** @return the voltage currently being applied to the motor. */
  Voltage getVoltage();

  /** Closes the motor. (Duplicates the signature for java.lang.Closeable.) */
  void close() throws IOException;

  /**
   * Wraps the provided WPILib controller with this interface.
   *
   * @param controller the ("raw") motor controller being wrapped
   * @return a wrapped controller, implementing IMotorControllerPlus
   */
  static IMotorControllerPlus forPWMMotorController(
      PWMMotorController controller) {
    return new MotorControllerPlus(controller,
        () -> Volts.of(controller.getVoltage()), () -> controller.close());
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

    /**
     * Constructor.
     *
     * @param controller      an underlying (wrapped/"raw") motor controller
     * @param voltageSupplier spplies the voltage being sent to the motor
     * @param closer          used to close the underlying motor controller
     */
    public MotorControllerPlus(MotorController controller,
        Supplier<Voltage> voltageSupplier, Closeable closer) {
      m_controller = controller;
      m_voltageSupplier = voltageSupplier;
      m_closer = closer;
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
  }
}
