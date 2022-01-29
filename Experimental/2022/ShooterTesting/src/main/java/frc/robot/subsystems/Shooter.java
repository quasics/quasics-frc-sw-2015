// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Sample code for a shooter with PID control, allowing adjustments for
 * constraints via the Smart Dashboard.
 * 
 * Based on an <a
 * href=
 * "https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java">example
 * from Rev Robotics</a>.
 */
public class Shooter extends SubsystemBase {

  /** The motor being controlled by the system. */
  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

  /** Encoder, used to report current stats from within periodic(). */
  private final RelativeEncoder encoder = motor.getEncoder();

  /** PID controller used to try to get us to/hold us at the target speed. */
  private SparkMaxPIDController pidController = motor.getPIDController();

  /** Target speed (in RPM). */
  private double targetSpeedRPM = 0;

  /** Proportional speed constant. */
  private double kP = 6e-5;
  /** Integral speed constant. */
  private double kI = 0;
  /** Derivative speed constant. */
  private double kD = 0;
  /**
   * I-zone constant, specifying the range the |error| must be within for the
   * integral constant to take effect.
   */
  private double kIz = 0;
  /** Feed-forward constant. */
  private double kFF = 0.0002;

  public static final double MAX_RPM = 5700;

  public Shooter() {
    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);

    // Sets the min/max output for closed-loop control (effectively a pair of
    // "clamp" values for the output from the PID controller).
    pidController.setOutputRange(-1, +1);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Current speed (RPM)", 0);
    SmartDashboard.putNumber("Target speed (RPM)", targetSpeedRPM);
    SmartDashboard.putString("Max speed (RPM)", Integer.toString((int) MAX_RPM));
  }

  public void stop() {
    targetSpeedRPM = 0;
    SmartDashboard.putNumber("Target speed (RPM)", 0);

    // Don't spin it down: just *stop* it.
    motor.set(0);
  }

  public void setSpeed(double rpm) {
    targetSpeedRPM = rpm;
    SmartDashboard.putNumber("Target speed (RPM)", targetSpeedRPM);

    pidController.setReference(targetSpeedRPM, CANSparkMax.ControlType.kVelocity);
  }

  public double getTargetSpeed() {
    return targetSpeedRPM;
  }

  public double getSpeed() {
    return encoder.getVelocity();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);

    // Update current velocity on the SmartDashboard.
    SmartDashboard.putNumber("Current speed (RPM)", encoder.getVelocity());

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      pidController.setFF(ff);
      kFF = ff;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
