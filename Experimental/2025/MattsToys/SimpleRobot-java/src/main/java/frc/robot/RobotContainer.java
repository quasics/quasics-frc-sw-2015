// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.LogitechGamePad;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.RaiseElevator;
import frc.robot.subsystems.AbstractElevator;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.subsystems.simulations.SimDrivebase;
import frc.robot.subsystems.simulations.SimulatedElevator;
import frc.robot.subsystems.simulations.SimulatedVision;
import frc.robot.utils.DeadbandEnforcer;

public class RobotContainer {
  // Subsystems
  final IVision m_vision = new SimulatedVision();
  private final IDrivebase m_drivebase = new SimDrivebase();
  final AbstractElevator m_elevator = new SimulatedElevator();

  // Controllers
  private final Joystick m_driveController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);

  public RobotContainer() {
    configureArcadeDrive();
    configureBindings();
  }

  private void configureArcadeDrive() {
    final DeadbandEnforcer deadbandEnforcer = new DeadbandEnforcer(Constants.DriveTeam.DRIVER_DEADBAND);
    Supplier<Double> forwardSupplier;
    Supplier<Double> rotationSupplier;

    if (Robot.isReal()) {
      // Configure the real robot.
      //
      // Note that we're inverting the values because Xbox controllers return
      // negative values when we push forward.
      forwardSupplier = () -> -deadbandEnforcer
          .limit(m_driveController.getRawAxis(LogitechGamePad.LeftYAxis));
      rotationSupplier = () -> -deadbandEnforcer
          .limit(m_driveController
              .getRawAxis(LogitechGamePad.RightXAxis));
    } else {
      // Configure the simulated robot
      //
      // Note that we're assuming a keyboard-based controller is actually being
      // used in the simulation environment (for now), and thus we want to use
      // axis 0&1 (from the "Keyboard 0" configuration).
      forwardSupplier = () -> deadbandEnforcer
          .limit(m_driveController.getRawAxis(0));
      rotationSupplier = () -> -deadbandEnforcer
          .limit(m_driveController.getRawAxis(1));
    }

    m_drivebase.asSubsystem().setDefaultCommand(new ArcadeDrive(m_drivebase, forwardSupplier,
        rotationSupplier));
  }

  private void configureBindings() {
    // TODO: Configure any bindings as required.
  }

  public Command getAutonomousCommand() {
    // Simple demo command to drive forward while raising the elevator.
    return new ParallelCommandGroup(
        new DriveForDistance(m_drivebase, .50, Meters.of(3)),
        new RaiseElevator(m_elevator));
    // return Commands.print("No autonomous command configured");
  }
}
