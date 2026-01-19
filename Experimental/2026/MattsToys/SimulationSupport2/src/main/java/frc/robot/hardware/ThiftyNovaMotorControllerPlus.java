package frc.robot.hardware;

import static edu.wpi.first.units.Units.Volts;

import java.io.Closeable;
import java.io.IOException;

import com.thethriftybot.devices.ThriftyNova;

public class ThiftyNovaMotorControllerPlus extends IMotorControllerPlus.MotorControllerPlus {
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
