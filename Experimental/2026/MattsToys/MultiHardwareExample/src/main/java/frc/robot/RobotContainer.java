// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.hardware.actuators.IMotorControllerPlus;
import frc.robot.hardware.actuators.TalonMotorControllerPlus;
import frc.robot.hardware.sensors.TalonEncoderWrapper;
import frc.robot.hardware.sensors.TrivialEncoder;
import frc.robot.subsystems.implementation.SingleMotorThing;
import frc.robot.subsystems.interfaces.ISingleMotorThing;
import frc.robot.subsystems.real.SingleMotorThingNova;
import frc.robot.subsystems.real.SingleMotorThingSpark;
import frc.robot.subsystems.real.SingleMotorThingTalon;
import frc.robot.subsystems.simulation.SingleMotorThingSim;
import frc.robot.subsystems.SingleMotorThingGroup;

public class RobotContainer {
  /** Supported hardware configurations. */
  enum HardwareConfig {
    /** Simulated motor only. */
    Simulated,
    /** SparkMax motor controller (live). */
    Spark,
    /** Paired SparkMax motor controllers (live). */
    SparkPair,
    /** TalonFX motor controller (live). */
    Talon,
    /**
     * TalonFX motor controller, constructed directly from the basic
     * SingleMotorThing class (live).
     */
    TalonDirect,
    /** Thrifty Nova motor controller (live). */
    Thrifty,
    /** Victor motor controller (live). */
    Victor,
  }

  public static final int RIGHT_INTAKE_DEPLOYMENT_ID = 6;
  public static final int LEFT_INTAKE_DEPLOYMENT_ID = 7;

  /** Selected hardware configuration. */
  final HardwareConfig m_hardware = Robot.isSimulation() ? HardwareConfig.Simulated : HardwareConfig.Spark;

  // Sets up a "single motor thing", based on the selected hardware
  // configuration.
  final ISingleMotorThing m_singleMotorThing = switch (m_hardware) {
    case Simulated -> new SingleMotorThingSim();
    case Spark -> new SingleMotorThingSpark(5, true);
    case SparkPair -> {
      // Generate an example of a pair of motors being treated as a single (logical)
      // entity (e.g., paired motors responsible for extending/retracting an intake,
      // mounted in opposite directions from each other).
      yield new SingleMotorThingGroup(
          new SingleMotorThingSpark(
              RIGHT_INTAKE_DEPLOYMENT_ID, true),
          new SingleMotorThingSpark(LEFT_INTAKE_DEPLOYMENT_ID, false));
    }
    case Talon -> new SingleMotorThingTalon(6);
    case TalonDirect -> {
      final TalonFX talon = new TalonFX(6);
      final IMotorControllerPlus motorController = new TalonMotorControllerPlus(talon);
      final TrivialEncoder encoder = new TalonEncoderWrapper(talon, Inches.of(6));
      yield new SingleMotorThing(new SingleMotorThing.ConstructionData(motorController, encoder));
    }
    case Thrifty -> new SingleMotorThingNova(5);
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
    System.out.println("Hardware configuration: " + m_hardware);

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
