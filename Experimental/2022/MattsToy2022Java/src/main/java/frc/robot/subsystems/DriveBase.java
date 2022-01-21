// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {

  private CANSparkMax leftRear = new CANSparkMax(Constants.MotorIds.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax rightRear = new CANSparkMax(Constants.MotorIds.RIGHT_REAR_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private CANSparkMax leftFront = new CANSparkMax(Constants.MotorIds.LEFT_FRONT_DRIVE_MOTOR_ID,
      MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(Constants.MotorIds.RIGHT_FRONT_DRIVE_MOTOR_ID,
      MotorType.kBrushless);

  private RelativeEncoder leftEncoder = leftRear.getEncoder();
  private RelativeEncoder rightEncoder = rightRear.getEncoder();

  private MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftRear);
  private MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightRear);

  /** Creates a new DriveBase. */
  public DriveBase() {
    this.setName("DriveBase");
    // TODO: Add code to configure encoders.
  }

  public void stop() {
    setPower(0, 0);
  }

  public void setPower(double leftPercent, double rightPercent) {
    leftMotors.set(leftPercent);
    rightRear.set(rightPercent);
  }

  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
