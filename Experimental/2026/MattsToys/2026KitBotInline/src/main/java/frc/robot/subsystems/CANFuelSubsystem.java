// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class CANFuelSubsystem extends SubsystemBase {
  private final MotorController feederRoller;
  private final MotorController intakeLauncherRoller;

  final static String INTAKING_FEEDER_VOLTAGE_KEY =
      "Intaking feeder roller value";
  final static String INTAKING_INTAKE_VOLTAGE_KEY =
      "Intaking intake roller value";
  final static String LAUNCHING_FEEDER_VOLTAGE_KEY =
      "Launching feeder roller value";
  final static String LAUNCHING_LAUNCHER_VOLTAGE_KEY =
      "Launching launcher roller value";
  final static String SPIN_UP_FEEDER_VOLTAGE_KEY =
      "Spin-up feeder roller value";

  private static MotorController allocateIntakeLauncherRoller() {
    if (Constants.USE_SPARK_MAX_OVER_CAN) {
      SparkMax rawIntakeLauncherRoller =
          new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
      // create the configuration for the launcher roller, set a current limit,
      // set the motor to inverted so that positive values are used for both
      // intaking and launching, and apply the config to the controller
      SparkMaxConfig launcherConfig = new SparkMaxConfig();
      launcherConfig.inverted(true);
      launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
      rawIntakeLauncherRoller.configure(launcherConfig,
          ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      return rawIntakeLauncherRoller;
    } else {
      PWMSparkMax rawRoller = new PWMSparkMax(INTAKE_LAUNCHER_MOTOR_ID);
      rawRoller.setInverted(true);
      return rawRoller;
    }
  }

  private static MotorController allocateFeederRoller() {
    if (Constants.USE_SPARK_MAX_OVER_CAN) {
      SparkMax rawFeederRoller =
          new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);

      // create the configuration for the feeder roller, set a current limit and
      // apply the config to the controller
      SparkMaxConfig feederConfig = new SparkMaxConfig();
      feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
      rawFeederRoller.configure(feederConfig, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

      return rawFeederRoller;
    } else {
      PWMSparkMax rawFeederRoller = new PWMSparkMax(FEEDER_MOTOR_ID);
      rawFeederRoller.setInverted(false);
      return rawFeederRoller;
    }
  }

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = allocateIntakeLauncherRoller();
    feederRoller = allocateFeederRoller();

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to
    // allow you to tune the values easily, and then replace the values in
    // Constants.java with your new values. For more information, see the
    // Software Guide.
    SmartDashboard.putNumber(
        INTAKING_FEEDER_VOLTAGE_KEY, INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(
        INTAKING_INTAKE_VOLTAGE_KEY, INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber(
        LAUNCHING_FEEDER_VOLTAGE_KEY, LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber(
        LAUNCHING_LAUNCHER_VOLTAGE_KEY, LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber(
        SPIN_UP_FEEDER_VOLTAGE_KEY, SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber(
        INTAKING_FEEDER_VOLTAGE_KEY, INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(
        INTAKING_INTAKE_VOLTAGE_KEY, INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake.
  // Uses the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller.setVoltage(-1
        * SmartDashboard.getNumber(
            INTAKING_FEEDER_VOLTAGE_KEY, INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(-1
        * SmartDashboard.getNumber(
            INTAKING_INTAKE_VOLTAGE_KEY, INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller.setVoltage(SmartDashboard.getNumber(
        LAUNCHING_FEEDER_VOLTAGE_KEY, LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(
        LAUNCHING_LAUNCHER_VOLTAGE_KEY, LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederRoller.setVoltage(SmartDashboard.getNumber(
        SPIN_UP_FEEDER_VOLTAGE_KEY, SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(
        LAUNCHING_LAUNCHER_VOLTAGE_KEY, LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A command factory to turn the spinUp method into a command that requires
  // this subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires
  // this subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
