// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ArcadeDrive;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeRoller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private boolean m_switchDrive = false;

  private final Drivebase m_drivebase = new Drivebase();
  private final Climbers m_climbers = new Climbers();
  private final IntakeRoller m_intakeRollers = new IntakeRoller();
  private final Shooter m_shooter = new Shooter();

  private final boolean ARCADE_DRIVE = true; // false for tank drive

  Supplier<Double> m_tankDriveLeftStick;
  Supplier<Double> m_tankDriveRightStick;
  Supplier<Double> m_arcadeDriveLeftStick;
  Supplier<Double> m_arcadeDriveRightStick;

  private final Joystick m_driverController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);
  private final Joystick m_operatorController = new Joystick(Constants.DriveTeam.OPERATOR_JOYSTICK_ID);

  Trigger switchDriveTrigger;

  SendableChooser m_autonomousOptions = new SendableChooser();

  private final double DEADBAND_CONSTANT = 0.04;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    addButtonsToSmartDashboard();
  }



  private void addAutonomousCommandsToSmartDashboard() {
    // TODO URGENT m_autonomousOptions.setDefaultOption(Constants.AutonomousSelectedOperation.doNothing);
  }

  private void addButtonsToSmartDashboard() {
    SmartDashboard.putData("set motor 6V", new InstantCommand(() -> m_drivebase.setVoltages(6, 6)));
    SmartDashboard.putData("Reset odometry", new InstantCommand(() -> m_drivebase.resetOdometry()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
    */

  private double getDriverAxis(int controllerCode) {
    double axis = m_driverController.getRawAxis(controllerCode);
    // dead band enforcer
    return (Math.abs(axis) > DEADBAND_CONSTANT) ? axis : 0;
  }
  
  private final SlewRateLimiter m_leftSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rightSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_arcadeSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(3);

  private void configureBindings() {
    m_tankDriveLeftStick = () -> {
      double axis = -getDriverAxis(Constants.LogitechGamePad.LeftYAxis);
      if (m_switchDrive) {
        double joystickPercentage = -axis *getDriveSpeedScalingFactor();
        return m_leftSpeedLimiter.calculate(joystickPercentage);
      }
      else {
        double joystickPercentage = axis * getDriveSpeedScalingFactor();
        return m_leftSpeedLimiter.calculate(joystickPercentage);
      }
    };

    m_tankDriveRightStick = () -> {
      double axis = -getDriverAxis(Constants.LogitechGamePad.RightYAxis);
      if (m_switchDrive) {
        double joystickPercentage = -axis *getDriveSpeedScalingFactor();
        return m_rightSpeedLimiter.calculate(joystickPercentage);
      }
      else {
        double joystickPercentage = axis * getDriveSpeedScalingFactor();
        return m_rightSpeedLimiter.calculate(joystickPercentage);
      }
    };


    m_arcadeDriveLeftStick = () -> {
      double axis = -getDriverAxis(Constants.LogitechGamePad.LeftYAxis);
      if (m_switchDrive) {
        double joystickPercentage = -axis * getDriveSpeedScalingFactor();
        return m_arcadeSpeedLimiter.calculate(joystickPercentage);
      }
      else {
        double joystickPercentage = axis * getDriveSpeedScalingFactor();
        return m_arcadeSpeedLimiter.calculate(joystickPercentage);
      }
    };

    m_arcadeDriveRightStick = () -> {
      double axis = -getDriverAxis(Constants.LogitechGamePad.RightXAxis);
      double joystickPercentage = axis * getDriveSpeedScalingFactor();
      return m_rotationLimiter.calculate(joystickPercentage);
    };

    switchDriveTrigger = new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.BButton)).onTrue(
      new InstantCommand(() -> m_switchDrive = !m_switchDrive));
    
    if (ARCADE_DRIVE) {
      m_drivebase.setDefaultCommand(new ArcadeDrive(m_drivebase, m_arcadeDriveLeftStick, m_arcadeDriveRightStick));
    }
    else {
      m_drivebase.setDefaultCommand(new TankDrive(m_drivebase, m_tankDriveLeftStick, m_tankDriveRightStick));
    }
  }

  private double getDriveSpeedScalingFactor() {
    final boolean isTurbo = m_driverController.getRawButton(Constants.LogitechGamePad.RightShoulder);
    final boolean isTurtle = m_driverController.getRawButton(Constants.LogitechGamePad.LeftShoulder);

    if (isTurbo) {
      return Constants.RobotSpeedScaling.TURBO_MODE_SPEED_SCALING;
    } else if (isTurtle) {
      return Constants.RobotSpeedScaling.TURTLE_MODE_SPEED_SCALING;
    } else {
      return Constants.RobotSpeedScaling.NORMAL_MODE_SPEED_SCALING;
    }
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}
