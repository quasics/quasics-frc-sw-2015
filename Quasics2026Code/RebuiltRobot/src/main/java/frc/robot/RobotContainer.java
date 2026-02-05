// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LinearSpeedCommand;
import frc.robot.subsystems.AbstractDrivebase;
import frc.robot.subsystems.RealDrivebase;
import frc.robot.subsystems.SimulatedVision;
import frc.robot.subsystems.SimulationDrivebase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.interfaces.IVision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private AbstractDrivebase m_drivebase = Robot.isReal() ? new RealDrivebase() : new SimulationDrivebase();
  private final IVision m_vision = (Robot.isReal()) ? new Vision(m_drivebase::getEstimatedPose)
      : new SimulatedVision(m_drivebase::getEstimatedPose);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
   * with an arbitrary predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link
   * edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers
   * or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // TODO: Schedule ArcadeDrive as our default command using
    // m_drivebase.setDefaultCommand(new ArcadeDrive(...));

    // Syntax for speed suppliers:
    // () -> m_driverController.getRawAxis(0)

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    LinearSpeedCommand setLinearSpeed = new LinearSpeedCommand(m_drivebase);
    SmartDashboard.putData("LinearSpeedCommand", setLinearSpeed);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Implement functionality for autonomous mode.
    return Commands.print("We should do something in auto mode....");
  }
}
