package first.robot.mechanisms;

import static org.wpilib.units.Units.Seconds;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.drivers.motor.PWMSparkMax;
import org.wpilib.hardware.motor.PWMMotorController;

public class DriveBase implements Mechanism {
  final PWMMotorController leftMotor = new PWMSparkMax(1);
  final PWMMotorController rightMotor = new PWMSparkMax(2);

  public DriveBase() {
    // Initialize your drive base here
  }

  private void stop() {
    setMotorSpeeds(0, 0);
  }

  private void setMotorSpeeds(double leftSpeed, double rightSpeed) {
    leftMotor.setThrottle(leftSpeed);
    rightMotor.setThrottle(rightSpeed);
  }

  // Add methods to control the drive base, such as driving forward, turning, etc.
  public Command driveForward(double speed, Supplier<Boolean> condition, String name) {
    return run(coroutine -> {
          setMotorSpeeds(speed, speed);
          while (condition.get()) {
            coroutine.yield();
          }
        })
        .whenExited(this::stop)
        .named(name);
  }

  public Command driveForward(double speed) {
    return driveForward(speed, () -> true, "Drive Forward");
  }

  public Command driveForward(double speed, double durationSeconds) {
    return driveForward(speed, () -> false, "Drive Forward (" + durationSeconds + "s)")
        .withTimeout(Seconds.of(durationSeconds));
  }
}
