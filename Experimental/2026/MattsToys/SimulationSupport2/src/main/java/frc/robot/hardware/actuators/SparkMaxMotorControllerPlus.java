package frc.robot.hardware.actuators;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkMaxMotorControllerPlus
    extends IMotorControllerPlus.MotorControllerPlus {
  public SparkMaxMotorControllerPlus(SparkMax controller) {
    super(controller,
        () -> Volts.of(controller.getAppliedOutput()),
        () -> controller.close(),
        // Is brake mode supported?
        true,
        // "Set brake mode (on/off)"
        (Boolean b) -> {
          SparkMaxConfig config = new SparkMaxConfig();
          config.idleMode(b ? IdleMode.kBrake : IdleMode.kCoast);
          controller.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        });
  }
}
