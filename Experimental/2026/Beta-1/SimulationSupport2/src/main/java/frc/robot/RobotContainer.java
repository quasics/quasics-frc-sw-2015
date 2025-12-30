// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TankDrive;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IElevator;
import frc.robot.subsystems.interfaces.IElevator.ElevatorPosition;
import frc.robot.subsystems.simulated.SimDrivebase;
import frc.robot.util.DriverJoystickWrapper;

public class RobotContainer {
  /** Whether to use arcade drive or tank drive for robot navigation. */
  private static final boolean USE_ARCADE_DRIVE = true;

  /** The drivebase subsystem. */
  private final IDrivebase drivebase = Robot.isReal() ? new Drivebase() : new SimDrivebase();

  private final IElevator elevator = new frc.robot.subsystems.simulated.SimElevator();

  /** The driver joystick wrapper. */
  private final DriverJoystickWrapper m_driverWrapper =
      new DriverJoystickWrapper(OperatorConstants.DRIVER_JOYSTICK_ID,
          // Only load from/save to preferences when in simulation
          Robot.isSimulation());

  /** The autonomous command chooser. */
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  /** Constructor. */
  public RobotContainer() {
    configureDriving();
    configureBindings();

    SmartDashboard.putData("Cmd: Elevator up", new InstantCommand(() -> {
      elevator.setTargetPosition(ElevatorPosition.HIGH);
    }, elevator.asSubsystem()));
    SmartDashboard.putData("Cmd: Elevator down", new InstantCommand(() -> {
      elevator.setTargetPosition(ElevatorPosition.BOTTOM);
    }, elevator.asSubsystem()));
    SmartDashboard.putData("Cmd: Elevator stop",
        new InstantCommand(() -> { elevator.stop(); }, elevator.asSubsystem()));
  }

  /** Configures the driving behavior. */
  private void configureDriving() {
    m_driverWrapper.setDeadbandThreshold(OperatorConstants.DEADBAND_THRESHOLD);
    SlewRateLimiter limiter1 = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);
    SlewRateLimiter limiter2 = new SlewRateLimiter(OperatorConstants.MAX_SLEW_RATE);
    if (USE_ARCADE_DRIVE) {
      drivebase.asSubsystem().setDefaultCommand(new ArcadeDrive(drivebase,
          ()
              -> limiter1.calculate(m_driverWrapper.getArcadeForward()),
          () -> limiter2.calculate(m_driverWrapper.getArcadeRotation())));
    } else {
      drivebase.asSubsystem().setDefaultCommand(new TankDrive(drivebase,
          ()
              -> limiter1.calculate(m_driverWrapper.getTankLeft()),
          () -> limiter2.calculate(m_driverWrapper.getTankRight())));
    }
  }

  private void configureBindings() {
    // Set up autonomous command chooser
    autoCommandChooser.setDefaultOption(
        "No Auto", Commands.print("No autonomous command configured"));
    autoCommandChooser.addOption("Do something", Commands.print("Do something"));
    SmartDashboard.putData("Autonomous Command", autoCommandChooser);
  }

  /** Returns the command to run in autonomous mode. */
  public Command getAutonomousCommand() {
    if (autoCommandChooser.getSelected() != null) {
      return autoCommandChooser.getSelected();
    }
    return Commands.print("No selection found for autonomous command");
  }
}