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

public class Shooter extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

  private final RelativeEncoder encoder = motor.getEncoder();

  private SparkMaxPIDController pidController = motor.getPIDController();

  private double targetSpeedRPM = 0;

  private double kP = 6e-5;
  private double kI = 0;
  private double kD = 0;
  private double kIz = 0;
  private double kFF = 0.000015;
  private double kMaxOutput = -1;
  private double kMinOutput = +1;
  public static final double MAX_RPM = 5700;

  public Shooter() {
    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
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
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

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
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
