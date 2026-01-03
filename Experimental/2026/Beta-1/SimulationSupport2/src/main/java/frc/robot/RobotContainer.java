// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.TankDrive;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.interfaces.IDrivebasePlus;
import frc.robot.subsystems.interfaces.IElevator;
import frc.robot.subsystems.interfaces.IElevator.ElevatorPosition;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.simulated.SimDrivebase;
import frc.robot.util.DriverJoystickWrapper;
import frc.robot.util.SysIdGenerator;
import frc.robot.util.SysIdGenerator.DrivebaseProfilingMode;
import java.util.List;

public class RobotContainer {
  /** Whether to use arcade drive or tank drive for robot navigation. */
  private static final boolean USE_ARCADE_DRIVE = true;

  /** The drivebase subsystem. */
  private final IDrivebasePlus drivebase = Robot.isReal() ? new Drivebase() : new SimDrivebase();

  /** The elevator subsystem.  (At present, always simulated.) */
  private final IElevator elevator = new frc.robot.subsystems.simulated.SimElevator();

  /** The arm subsystem.  (At present, always simulated.) */
  private final ISingleJointArm arm = new frc.robot.subsystems.simulated.SimArm();

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
    configureSysIdCommands();
    configureElevatorCommands();
    configureArmCommands();
  }

  private void configureArmCommands() {
    // Note that the delays will not be enough to get the arm *absolutely* into position,
    // but that's OK: I'm just trying to make it wave back and forth....
    Command waveCommand =
        // Move out
        new SequentialCommandGroup(
            new InstantCommand(
                () -> { arm.setTargetPosition(arm.getArmOutAngle()); }, arm.asSubsystem()),
            new PrintCommand("Waiting for out"), new WaitCommand(2),
            new InstantCommand(
                () -> { arm.setTargetPosition(arm.getArmUpAngle()); }, arm.asSubsystem()),
            new PrintCommand("Waiting for up"), new WaitCommand(2))
            .repeatedly();
    SmartDashboard.putData("Cmd: Arm out", new InstantCommand(() -> {
      arm.setTargetPosition(arm.getArmOutAngle());
    }, arm.asSubsystem()));
    SmartDashboard.putData("Cmd: Arm up", new InstantCommand(() -> {
      arm.setTargetPosition(arm.getArmUpAngle());
    }, arm.asSubsystem()));
    SmartDashboard.putData("Cmd: Arm wave", waveCommand);
    SmartDashboard.putData(
        "Cmd: Arm stop", new InstantCommand(() -> { arm.stop(); }, arm.asSubsystem()));
  }

  private void configureElevatorCommands() {
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

  private void configureSysIdCommands() {
    SmartDashboard.putData("Cmd: DynamicFwd",
        SysIdGenerator.sysIdDynamic(drivebase, DrivebaseProfilingMode.Linear, Direction.kForward));
    SmartDashboard.putData("Cmd: DynamicRev",
        SysIdGenerator.sysIdDynamic(drivebase, DrivebaseProfilingMode.Linear, Direction.kReverse));
    SmartDashboard.putData("Cmd: QStaticFwd",
        SysIdGenerator.sysIdQuasistatic(
            drivebase, DrivebaseProfilingMode.Linear, Direction.kForward));
    SmartDashboard.putData("Cmd: QStaticRev",
        SysIdGenerator.sysIdQuasistatic(
            drivebase, DrivebaseProfilingMode.Linear, Direction.kReverse));
  }

  private void configureBindings() {
    // Set up autonomous command chooser
    autoCommandChooser.setDefaultOption(
        "No Auto", Commands.print("No autonomous command configured"));
    autoCommandChooser.addOption("Do something", Commands.print("Do something"));
    autoCommandChooser.addOption("Sample trajectory", getSampleTrajectoryCommand());
    SmartDashboard.putData("Autonomous Command", autoCommandChooser);
  }

  /** Returns the command to run in autonomous mode. */
  public Command getAutonomousCommand() {
    if (autoCommandChooser.getSelected() != null) {
      return autoCommandChooser.getSelected();
    }
    return Commands.print("No selection found for autonomous command");
  }

  Command getSampleTrajectoryCommand() {
    // Note: all of the following are *example* values only, and would need to be
    // appropriately generated (e.g., via SysId profiling, etc.).
    final double ksVolts = 0.014183;
    final double kvVoltSecondsPerMeter = 1.9804;
    final double kaVoltSecondsSquaredPerMeter = 0.19169;
    final double kMaxVoltage = 10;
    final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(3);

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
        Drivebase.KINEMATICS, kMaxVoltage);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Drivebase.MAX_SPEED, maxAcceleration)
                                  // Add kinematics to ensure max speed is actually obeyed
                                  .setKinematics(Drivebase.KINEMATICS)
                                  // Apply the voltage constraint
                                  .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
    return new FollowTrajectoryCommand(drivebase, exampleTrajectory);
  }
}