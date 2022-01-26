// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

  private final RelativeEncoder encoder = motor.getEncoder();

  public Shooter() {
  }

  public void stop() {
    setPower(0);
  }

  public void setPower(double percentage) {
    motor.set(percentage);
  }

  public double getSpeed() {
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
