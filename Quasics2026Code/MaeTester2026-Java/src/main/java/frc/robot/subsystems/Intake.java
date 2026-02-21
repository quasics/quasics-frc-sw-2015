// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConditionalConstants;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public static final double MOTOR_OFF_POWER = 0;
  public static final double MOTOR_SLOW_POWER = 0.25;
  public static final double MOTOR_FAST_POWER = 0.75;
  public static final double MOTOR_FULL_POWER = 1.0;

  private WPI_VictorSPX m_intakeMotor = null;
  private WPI_VictorSPX m_conveyorMotor = null;

  private boolean isHardwareDisabled() {
    return ConditionalConstants.SALLY;
  }

  /** Creates a new Intake. */
  public Intake() {
    setSubsystem("Intake");
    if (isHardwareDisabled()) {
      return;
    }
    m_intakeMotor =
        new WPI_VictorSPX(Constants.CANBusIds.VictorSPXIds.IntakeMotor);
    m_conveyorMotor =
        new WPI_VictorSPX(Constants.CANBusIds.VictorSPXIds.ConveyorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Intakes the ball at 1/4 speed. Theoretically, when limit switch is hit,
  // both Intake and Conveyor will stop.
  // TODO: Fix this: it shouldn't be written like this!!!!!
  // TODO(matt): Explain to people why.
  public void intakeBallOn() {
    if (!isBallInChamber()) {
      m_conveyorMotor.set(MOTOR_FAST_POWER);
      m_intakeMotor.set(MOTOR_SLOW_POWER);
    } else {
      if (isHardwareDisabled()) {
        return;
      }
      m_intakeMotor.set(MOTOR_OFF_POWER);
      m_conveyorMotor.set(MOTOR_OFF_POWER);
    }
  }

  public void intakeBallOff() {
    if (isHardwareDisabled()) {
      return;
    }
    m_intakeMotor.set(MOTOR_OFF_POWER);
    m_conveyorMotor.set(MOTOR_OFF_POWER);
  }

  public void conveyBallOn() {
    if (isHardwareDisabled()) {
      return;
    }
    m_conveyorMotor.set(MOTOR_FULL_POWER);
  }

  public void conveyBallOnSlow() {
    if (isHardwareDisabled()) {
      return;
    }
    m_conveyorMotor.set(MOTOR_SLOW_POWER);
  }

  public void conveyBallReverse() {
    if (isHardwareDisabled()) {
      return;
    }
    m_conveyorMotor.set(-MOTOR_SLOW_POWER);
  }

  public void conveyBallOff() {
    if (isHardwareDisabled()) {
      return;
    }
    m_conveyorMotor.set(MOTOR_OFF_POWER);
  }

  public void onlyIntakeOn() {
    if (isHardwareDisabled()) {
      return;
    }
    m_intakeMotor.set(MOTOR_FAST_POWER);
  }

  public void onlyIntakeReverse() {
    if (isHardwareDisabled()) {
      return;
    }
    m_intakeMotor.set(-MOTOR_FAST_POWER);
  }

  public void onlyIntakeOff() {
    if (isHardwareDisabled()) {
      return;
    }
    m_intakeMotor.set(MOTOR_OFF_POWER);
  }

  /** TODO: Implement this using beam break sensor. */
  public boolean isBallInChamber() {
    return false;
  }

  /**
   * Sets the speed for the ball pick-up mechanism.
   * @param percent
   *           a value from 1.0 (full forward) to -1.0 (full reverse); 0 ==
   *           stop.
   */
  public void setBallPickupSpeed(
      double percent) { // Cap the value of percent to -1.0 to +1.0.
    double useSpeed = Math.max(-1.0, Math.min(1.0, percent));
    if (isHardwareDisabled()) {
      return;
    }
    m_intakeMotor.set(useSpeed);
  }

  /**
   * Sets the speed for the conveyor used to transfer balls from the
   * "hopper" to the shooter.
   *
   * @param percent
   *           a value from 1.0 (full forward) to -1.0 (full reverse); 0 ==
   *           stop.
   */
  void setConveyorSpeed(
      double percent) { // Cap the value of percent to -1.0 to +1.0.
    double useSpeed = Math.max(-1.0, Math.min(1.0, percent));
    if (isHardwareDisabled()) {
      return;
    }
    m_conveyorMotor.set(useSpeed);
  }
}
