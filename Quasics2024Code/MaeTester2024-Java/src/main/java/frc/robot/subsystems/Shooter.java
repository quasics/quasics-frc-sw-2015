// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the shooter subsystem (both motor and servo for angle).
 *
 * Settings per AndyMark docs for the L16 Actuator/servo are defined at:
 * https://www.andymark.com/products/actuator-l16-r-50mm-stroke-35-1-6v
 */
public class Shooter extends SubsystemBase {
  public static final double POSITION_DELTA = 0.05;

  // Also from AndyMark docs (link above):
  //   For FTC:
  //      To drive fully out, the position is set to 0.82.
  //      To drive half way, the position is set to 0.5.
  //      To drive fully in, the position is set to 0.17
  //   For FRC:
  //      yourActuator.setSpeed(1.0); // to open
  //      yourActuator.setSpeed(-1.0);  // to close
  private static final double SERVO_RETRACTED_SPEED = -1.0;
  private static final double SERVO_EXTENDED_SPEED = +1.0;
  private static final double SERVO_POSITION_RANGE =
      SERVO_EXTENDED_SPEED - SERVO_RETRACTED_SPEED;

  /** Actual shooter: sending the ball out. */
  private TalonFX shootingMotor =
      new TalonFX(Constants.CANBusIds.TalonFXIds.ShootingMotor);

  /** Experimental: servo to adjust the shooting angle. */
  private Servo positionServo = new Servo(Constants.PwmIds.ShooterServo);

  /** Creates a new Shooter. */
  public Shooter() {
    setName("Shooter");

    // Settings per AndyMark docs for the L16 Actuator/servo; see:
    // https://www.andymark.com/products/actuator-l16-r-50mm-stroke-35-1-6v
    positionServo.setBoundsMicroseconds(2000000, 1800000, 1500000, 1200000,
                                        1000000);

    // Make sure that the shooter starts with the servo in a known position.
    SetServoPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /// Sets speed for the shooting motor (-1.0 to +1.0).
  public void SetSpeed(double speed) {
    shootingMotor.set(-speed); // Yeet the ball
  }

  /// Stops the shooting motor.
  public void Stop() {
    shootingMotor.set(0);
  }

  /** @return servo extension as a percentage of range (0.0 - 1.0) */
  public double GetServoPosition() {
    var rawPos = positionServo.getSpeed();
    var percentPos = (rawPos - SERVO_RETRACTED_SPEED) / SERVO_POSITION_RANGE;
    return percentPos;
  }

  /**
   * @param pos   percent extension, expressed as [0.0 - 1.0]
   */
  public void SetServoPosition(double pos) {
    final double cappedPercent = Math.min(1.0, Math.max(pos, 0.0));
    positionServo.setSpeed(SERVO_RETRACTED_SPEED +
                           (cappedPercent * SERVO_POSITION_RANGE));
  }

  /** Increases shooting angle by POSITION_DELTA on the linear servo. */
  public void IncrementPosition() {
    SetServoPosition(GetServoPosition() + POSITION_DELTA);
  }

  /** Decreases shooting angle by POSITION_DELTA on the linear servo. */
  public void DecrementPosition() {
    SetServoPosition(GetServoPosition() - POSITION_DELTA);
  }
}
