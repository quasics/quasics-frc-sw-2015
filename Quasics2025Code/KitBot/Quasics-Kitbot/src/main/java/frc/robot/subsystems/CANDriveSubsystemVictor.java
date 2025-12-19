// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;

// Class to drive the robot over CAN
public class CANDriveSubsystemVictor extends AbstractDrivebase {
  // https://www.frczero.org/electronics/components-and-sensors/victor-spx/
  private final WPI_VictorSPX leftLeader;
  private final WPI_VictorSPX leftFollower;
  private final WPI_VictorSPX rightLeader;
  private final WPI_VictorSPX rightFollower;

  private final DifferentialDrive drive;

  public CANDriveSubsystemVictor() {
    // create brushed motors for drive
    leftLeader = new WPI_VictorSPX(DriveConstants.LEFT_LEADER_ID);
    leftFollower = new WPI_VictorSPX(DriveConstants.LEFT_FOLLOWER_ID);
    rightLeader = new WPI_VictorSPX(DriveConstants.RIGHT_LEADER_ID);
    rightFollower = new WPI_VictorSPX(DriveConstants.RIGHT_FOLLOWER_ID);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    VictorSPXConfiguration config = new VictorSPXConfiguration();
    config.voltageCompSaturation = 12;
    // No current limiting for VictorSPX

    // Set configuration to follow leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // TODO: Which side should be inverted?

    leftFollower.configAllSettings(config);
    rightFollower.configAllSettings(config);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);
  }

  @Override
  public void periodic() {
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
    System.out.println("Victor execute" +xSpeed +" " +zRotation + " " + drive.isAlive() + " " + drive.isSafetyEnabled());

    // Get faults
    Faults faults = new Faults();
    leftLeader.getFaults(faults);

    System.out.println("Motor Output: " + leftLeader.getMotorOutputPercent());
    System.out.println("Bus Voltage: " + leftLeader.getBusVoltage());
    System.out.println("Control Mode: " + leftLeader.getControlMode());

    // Check for faults
    if (faults.hasAnyFault()) {
        System.out.println("FAULTS DETECTED:");
        System.out.println("Hardware Failure: " + faults.HardwareFailure);
        System.out.println("Forward Limit: " + faults.ForwardLimitSwitch);
        System.out.println("Reverse Limit: " + faults.ReverseLimitSwitch);
        System.out.println("Under Voltage: " + faults.UnderVoltage);
    }
  }
}
