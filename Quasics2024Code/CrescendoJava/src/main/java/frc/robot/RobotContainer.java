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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeRoller;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Drivebase m_drivebase = new Drivebase();
  private final Climbers m_climbers = new Climbers();
  private final IntakeRoller m_intakeRollers = new IntakeRoller();
  private final Shooter m_shooter = new Shooter();

  private final boolean ARCADE_DRIVE = true; // false for tank drive

  Supplier<Double> m_tankDriveLeftStick;
  Supplier<Double> m_tankDriveRightStick;
  Supplier<Double> m_arcadeDriveLeftStick;
  Supplier<Double> m_arcadeDriveRightStick;

  private Joystick m_driveController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {
    m_tankDriveLeftStick = () -> -m_driveController.getRawAxis(Constants.LogitechGamePad.LeftYAxis);
    m_tankDriveRightStick = () -> -m_driveController.getRawAxis(Constants.LogitechGamePad.RightYAxis);

    m_arcadeDriveLeftStick = () -> -m_driveController.getRawAxis(Constants.LogitechGamePad.LeftYAxis);
    m_arcadeDriveRightStick = () -> -m_driveController.getRawAxis(Constants.LogitechGamePad.RightXAxis);
    if (ARCADE_DRIVE) {
      m_drivebase.setDefaultCommand(new ArcadeDrive(m_drivebase, m_arcadeDriveLeftStick, m_arcadeDriveRightStick));
    }
    else {
      m_drivebase.setDefaultCommand(new TankDrive(m_drivebase, m_tankDriveLeftStick, m_tankDriveRightStick));
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
