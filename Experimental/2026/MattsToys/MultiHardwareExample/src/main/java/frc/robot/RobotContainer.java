// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.sensors.TrivialEncoder;
import frc.robot.subsystems.implementation.SingleMotorThing;
import frc.robot.subsystems.interfaces.ISingleMotorThing;
import frc.robot.subsystems.real.SingleMotorThingNova;
import frc.robot.subsystems.real.SingleMotorThingSpark;
import frc.robot.subsystems.real.SingleMotorThingTalon;
import frc.robot.subsystems.simulation.SingleMotorThingSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** Supported hardware configurations. */
  enum HardwareConfig {
    Simulated, Thrifty, Spark, Talon, Victor
  }

  /** Selected hardware configuration. */
  final HardwareConfig m_hardware = Robot.isSimulation() ? HardwareConfig.Simulated : HardwareConfig.Thrifty;

  // Sets up a "single motor thing", based on the selected hardware configuration.
  final ISingleMotorThing m_singleMotorThing = switch (m_hardware) {
    case Simulated -> new SingleMotorThingSim();
    case Spark -> new SingleMotorThingSpark();
    case Talon -> new SingleMotorThingTalon();
    case Thrifty -> new SingleMotorThingNova();
    case Victor ->
      // Sample of how to use the SingleMotorThing class without needing
      // to derive a class for hardware-specific setup.
      new SingleMotorThing(new SingleMotorThing.ConstructionData(
          // Use a Victor motor controller...
          new VictorSP(8),
          // ...and a bog-standard WPILib encoder.
          TrivialEncoder.forWpiLibEncoder(new Encoder(1, 2))));
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addPowerButton("Stop!", 0);
    addPowerButton("-25% power", -.25);
    addPowerButton("-50% power", -.5);
    addPowerButton("-100% power", -1.0);
    addPowerButton("+25% power", +.25);
    addPowerButton("+50% power", +.5);
    addPowerButton("+100% power", +1.0);

    SmartDashboard.putData(m_singleMotorThing.asSendable());
  }

  private void addPowerButton(String label, double percent) {
    SmartDashboard.putData(label,
        new FunctionalCommand(
            // onInit (can't be null)
            () -> {
              m_singleMotorThing.setSpeed(percent);
            },
            // onExecute (can't be null)
            () -> {
              // No-op: speed was set in initialization
            },
            // onEnd (can't be null)
            (Boolean b) -> {
              m_singleMotorThing.stop();
            },
            // isFinished (can't be null)
            () -> false,
            // Dependency
            m_singleMotorThing.asSubsystem()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
