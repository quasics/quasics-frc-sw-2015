// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A simple flywheel subsystem that uses a SparkMax and WPILib's
 * SimpleMotorFeedforward to control the flywheel's velocity.
 *
 * This is a "live" subsystem, meaning it's meant for testing and
 * experimentation. It may not be fully optimized or robust, but it should
 * provide a good starting point for understanding how to use feedforward
 * control with a SparkMax.
 */
public class Flywheel extends SubsystemBase {
  private final SparkMax motor;
  private final SimpleMotorFeedforward feedforward;

  // TODO: These constants should be determined experimentally using SysId or
  // another approach.
  private static final double kS = 0.1; // Volts (can often be ignored for flywheel velocity control)
  private static final double kV = 0.002; // Volts per RPM
  private static final double kA = 0.0001; // Volts per RPM
  private static final double kP = 0.0001; // Proportional gain for onboard loop

  public Flywheel(int deviceId) {
    motor = new SparkMax(deviceId, MotorType.kBrushless);

    // Initialize WPILib Feedforward with SysId constants (Volts)
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    SparkMaxConfig config = new SparkMaxConfig();

    // Set ONLY kP for the onboard loop. (If we see a persistant small gap between
    // the target RPM and what we're actually getting out of the subsystem, then I
    // can try adding a *tiny* bit of kI.)
    //
    // Note: I am leaving kFF at 0 because I am calculating it manually.
    config.closedLoop.p(kP, ClosedLoopSlot.kSlot0);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setRPM(double targetRPM) {
    // Calculate the necessary voltage using WPILib's helper
    final double ffVoltage = feedforward.calculate(targetRPM);

    // Send target to SparkMax and "inject" the Feedforward voltage
    // This tells the SparkMax: "Aim for this RPM, and start with this many Volts"
    // The SparkMax will then use its onboard PID controller to adjust the voltage
    // it applies to the motor, starting from the FF voltage, to try to reach the
    // target RPM.
    //
    // Note: The FF voltage is not a "setpoint" for the PID loop; it's more of a
    // starting point that helps the motor get close to the target RPM faster, and
    // reduces the amount of work the PID controller has to do.
    //
    // TODO: Add error handling. (Or at least logging.)
    motor.getClosedLoopController().setSetpoint(
        targetRPM,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, // Slot ID
        ffVoltage, // The Arbitrary Feedforward value
        ArbFFUnits.kVoltage // Specify that the FF value is in Volts
    );
  }

  static final boolean kDebug = true;

  @Override
  public void periodic() {
    if (kDebug) {
      final double targetRPM = motor.getClosedLoopController().getSetpoint();
      final double currentRPM = motor.getEncoder().getVelocity();
      final double ffVoltage = feedforward.calculate(targetRPM);
      SmartDashboard.putNumber("Flywheel RPM", currentRPM);
      SmartDashboard.putNumber("Flywheel Target RPM", targetRPM);
      SmartDashboard.putNumber("Flywheel FF Voltage", ffVoltage);
    }
  }
}
