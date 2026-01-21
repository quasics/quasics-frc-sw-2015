// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class CANDriveSubsystem extends SubsystemBase {
  private record MotorSet(MotorController left, MotorController right) {}

  private final DifferentialDrive drive;

  private static MotorSet setupMotors() {
    if (Constants.USE_SPARK_MAX_OVER_CAN) {
      // create brushed motors for drive
      final SparkMax leftLeader =
          new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
      final SparkMax rightLeader =
          new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
      @SuppressWarnings("resource")
      final SparkMax leftFollower =
          new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
      @SuppressWarnings("resource")
      final SparkMax rightFollower =
          new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

      // Set can timeout. Because this project only sets parameters once on
      // construction, the timeout can be long without blocking robot operation.
      // Code which sets or gets parameters during operation may need a shorter
      // timeout.
      leftLeader.setCANTimeout(250);
      rightLeader.setCANTimeout(250);
      leftFollower.setCANTimeout(250);
      rightFollower.setCANTimeout(250);

      // Create the configuration to apply to motors. Voltage compensation
      // helps the robot perform more similarly on different
      // battery voltages (at the cost of a little bit of top speed on a fully
      // charged battery). The current limit helps prevent tripping breakers.
      SparkMaxConfig config = new SparkMaxConfig();
      config.voltageCompensation(12);
      config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

      // Set configuration to follow each leader and then apply it to
      // corresponding follower. Resetting in case a new controller is swapped
      // in and persisting in case of a controller reset due to breaker trip
      config.follow(leftLeader);
      leftFollower.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
      config.follow(rightLeader);
      rightFollower.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

      // Remove following, then apply config to right leader
      config.disableFollowerMode();
      rightLeader.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
      // Set config to inverted and then apply to left leader. Set Left side
      // inverted so that postive values drive both sides forward
      config.inverted(true);
      leftLeader.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);

      return new MotorSet(leftLeader, rightLeader);
    } else {
      PWMSparkMax leftLeader = new PWMSparkMax(LEFT_LEADER_ID);
      PWMSparkMax rightLeader = new PWMSparkMax(RIGHT_LEADER_ID);
      PWMSparkMax leftFollower = new PWMSparkMax(LEFT_FOLLOWER_ID);
      PWMSparkMax rightFollower = new PWMSparkMax(RIGHT_FOLLOWER_ID);
      leftLeader.addFollower(leftFollower);
      rightLeader.addFollower(rightFollower);

      leftLeader.setInverted(true);
      leftFollower.setInverted(true);
      rightLeader.setInverted(false);
      rightFollower.setInverted(false);

      return new MotorSet(leftLeader, rightLeader);
    }
  }

  public CANDriveSubsystem() {
    MotorSet motors = setupMotors();

    // set up differential drive class
    drive = new DifferentialDrive(motors.left, motors.right);
  }

  @Override
  public void periodic() {
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }
}
