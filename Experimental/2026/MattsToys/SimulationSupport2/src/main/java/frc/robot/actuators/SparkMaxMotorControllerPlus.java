package frc.robot.actuators;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;

public class SparkMaxMotorControllerPlus
    extends IMotorControllerPlus.MotorControllerPlus {
  public SparkMaxMotorControllerPlus(SparkMax controller) {
    super(controller,
        ()
            -> Volts.of(controller.getAppliedOutput()),
        () -> controller.close());
  }
}
