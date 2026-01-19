package frc.robot.actuators;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import java.io.Closeable;
import java.io.IOException;
import java.util.function.Supplier;

public interface IMotorControllerPlus extends MotorController {
  Voltage getVoltage();

  void close() throws IOException;

  static IMotorControllerPlus forPWMMotorController(
      PWMMotorController controller) {
    return new MotorControllerPlus(controller,
        () -> Volts.of(controller.getVoltage()), () -> controller.close());
  }

  public class MotorControllerPlus implements IMotorControllerPlus {
    final MotorController m_controller;
    final Supplier<Voltage> m_voltageSupplier;
    final Closeable m_closer;

    public MotorControllerPlus(MotorController controller,
        Supplier<Voltage> voltageSupplier, Closeable closer) {
      m_controller = controller;
      m_voltageSupplier = voltageSupplier;
      m_closer = closer;
    }

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
