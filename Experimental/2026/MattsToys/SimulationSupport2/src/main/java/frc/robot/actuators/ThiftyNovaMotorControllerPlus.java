package frc.robot.actuators;

import static edu.wpi.first.units.Units.Volts;

import com.thethriftybot.devices.ThriftyNova;
import java.io.Closeable;
import java.io.IOException;

public class ThiftyNovaMotorControllerPlus
    extends IMotorControllerPlus.MotorControllerPlus {
  public ThiftyNovaMotorControllerPlus(ThriftyNova controller) {
    super(controller, () -> Volts.of(controller.getVoltage()), new Closeable() {
      public void close() throws IOException {
        try {
          controller.close();
        } catch (Exception e) {
          throw new IOException(e);
        }
      }
    });
  }
}
