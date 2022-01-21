// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {

  private MotorController leftRear = new CANSparkMax(Constants.MotorIds.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
  private MotorController rightRear = new CANSparkMax(Constants.MotorIds.RIGHT_REAR_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private MotorController leftFront = new CANSparkMax(Constants.MotorIds.LEFT_FRONT_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private MotorController rightFront = new CANSparkMax(Constants.MotorIds.RIGHT_FRONT_DRIVE_MOTOR_ID,
      MotorType.kBrushless);

  /** Creates a new DriveBase. */
  public DriveBase() {
    this.setName("DriveBase");
  }

  public void stop() {
    setPower(0, 0);
  }

  public void setPower(double leftPercent, double rightPercent) {
    // TODO: Implement this.
  }

  public double getLeftEncoderPosition() {
    // TODO: Implement this.
    return 0;
  }

  public double getRightEncoderPosition() {
    // TODO: Implement this.
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
