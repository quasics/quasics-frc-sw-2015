// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import static frc.robot.util.RevSupportFunctions.configureAsNotFollowing;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.bases.BaseFlywheel;
import frc.robot.util.config.FlywheelConfig;

/**
 * A simple flywheel subsystem that uses a SparkMax and WPILib's
 * SimpleMotorFeedforward to control the flywheel's velocity.
 *
 * This is a "live" subsystem, meaning it's meant for testing and
 * experimentation. It may not be fully optimized or robust, but it should
 * provide a good starting point for understanding how to use feedforward
 * control with a SparkMax.
 */
public class SparkMaxFlywheel extends BaseFlywheel {
  private final SparkMax motor;

  public SparkMaxFlywheel(FlywheelConfig flywheelConfig) {
    super(flywheelConfig.feedForward());

    motor = new SparkMax(flywheelConfig.motorID(), MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    configureAsNotFollowing(config);

    // Note: I am leaving kFF at 0 because I am calculating it manually.
    config.closedLoop.p(flywheelConfig.pidConfig().kP(), ClosedLoopSlot.kSlot0);
    config.closedLoop.i(flywheelConfig.pidConfig().kI(), ClosedLoopSlot.kSlot0);
    config.closedLoop.d(flywheelConfig.pidConfig().kD(), ClosedLoopSlot.kSlot0);

    motor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setRPM(double targetRPM) {
    // Calculate the necessary voltage using WPILib's helper
    final double ffVoltage = feedforward.calculate(targetRPM);

    // Send target to SparkMax and "inject" the Feedforward voltage.
    // This tells the SparkMax: "Aim for this RPM, and start with this many
    // Volts." The SparkMax will then use its onboard PID controller to adjust
    // the voltage it applies to the motor, starting from the FF voltage, to try
    // to reach the target RPM.
    //
    // Note: The FF voltage is not a "setpoint" for the PID loop; it's more of a
    // starting point that helps the motor get close to the target RPM faster,
    // and reduces the amount of work the PID controller has to do.
    var result = motor.getClosedLoopController().setSetpoint(
        targetRPM, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, // Slot ID
        ffVoltage, // The Arbitrary Feedforward value
        ArbFFUnits.kVoltage // Specify that the FF value is in Volts
    );
    if (result != REVLibError.kOk) {
      // Handle error (e.g., log it)
      final String errorMessage = "Error setting flywheel RPM: " + result;
      DriverStation.reportError(errorMessage, false);
    }
  }

  @Override
  public double getCurrentRPM() {
    return motor.getEncoder().getVelocity();
  }

  @Override
  public double getSetpointRPM() {
    return motor.getClosedLoopController().getSetpoint();
  }
}
