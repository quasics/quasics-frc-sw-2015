// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.RunFloorPickup;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.
 * 
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Conveyor m_conveyor = new Conveyor();
  private final FloorIntake m_floorIntake = new FloorIntake();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureSmartDashboardButtons();
    configureButtonBindings();
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Pickup: +100%", new RunFloorPickup(m_floorIntake, 1.0));
    SmartDashboard.putData("Pickup: +90%", new RunFloorPickup(m_floorIntake, 0.9));
    SmartDashboard.putData("Pickup: +80%", new RunFloorPickup(m_floorIntake, 0.8));
    SmartDashboard.putData("Pickup: +70%", new RunFloorPickup(m_floorIntake, 0.7));
    SmartDashboard.putData("Pickup: +60%", new RunFloorPickup(m_floorIntake, 0.6));
    SmartDashboard.putData("Pickup: +50%", new RunFloorPickup(m_floorIntake, 0.5));
    SmartDashboard.putData("Pickup: -25%", new RunFloorPickup(m_floorIntake, -0.25));

    SmartDashboard.putData("Conveyor: +50%", new RunConveyor(m_conveyor, 0.5));
    SmartDashboard.putData("Conveyor: -25%", new RunConveyor(m_conveyor, -0.25));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PrintCommand("Do something autonomously....");
  }
}
