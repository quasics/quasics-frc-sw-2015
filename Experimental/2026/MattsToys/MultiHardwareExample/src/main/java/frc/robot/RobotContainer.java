// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.interfaces.ISingleMotorThing;
import frc.robot.subsystems.real.SingleMotorThingSpark;
import frc.robot.subsystems.real.SingleMotorThingTalon;
import frc.robot.subsystems.simulation.SingleMotorThingSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** Supported hardware configurations. */
  enum HardwareConfig { Simulated, Spark, Talon }

  /** Selected hardware configuration. */
  final HardwareConfig m_hardware =
      Robot.isSimulation() ? HardwareConfig.Simulated : HardwareConfig.Spark;

  // Sets up a "single motor thing", based on the selected hardware configuration.
  final ISingleMotorThing m_singleMotorThing = switch (m_hardware) {
    case Simulated -> new SingleMotorThingSim();
    case Spark -> new SingleMotorThingSpark();
    case Talon -> new SingleMotorThingTalon();
  };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("We should do something....");
  }
}
