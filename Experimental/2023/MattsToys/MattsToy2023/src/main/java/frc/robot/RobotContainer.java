// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.RobotSettings;
import frc.robot.utils.SpeedModifier;
import frc.robot.utils.SwitchModeSpeedSupplier;
import frc.robot.utils.TrajectoryCommandGenerator.DriveProfileData;
import frc.robot.utils.TrajectoryCommandGenerator.PIDConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // TODO: Modify this to try to load current settings from filesystem.
  private final Drivebase m_driveBase = new Drivebase(getDefaultSettings());

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * Used to manage "switch mode" (both providing the speed suppliers, and
   * managing current "switch mode" state).
   * 
   * This is set up by getTankDriveCommand().
   */
  private SwitchModeSpeedSupplier m_switchModeHandler;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //////////////////////////////////////
    // Drive base setup

    // "Late initialization"
    m_driveBase.finalizeSetup();

    // Default command for the subsystem.
    m_driveBase.setDefaultCommand(getTankDriveCommand());

    //////////////////////////////////////
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
    // Add a command (triggered by the "Y" button on the driver controller) to trigger "switch mode" change.
    Trigger yButton = m_driverController.y();
    final Command changeDirectionCommand = runOnce(() -> {
      if (m_switchModeHandler != null) {
        System.out.println("Switching heading mode");
        m_switchModeHandler.toggleSwitchMode();
      }
    });
    yButton
        .debounce(0.1)
        .onTrue(changeDirectionCommand);

    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
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

  /** Returns the robot settings for Sally (2022 robot). */
  private static RobotSettings getSettingsForSally() {
    return new RobotSettings(
        "Sally", // robotName
        Constants.TRACK_WIDTH_INCHES_SALLY / Constants.INCHES_PER_METER,
        Constants.DRIVE_BASE_GEAR_RATIO_SALLY,
        // TODO(mjh) Calibrate Sally's values for kS, kV, and kA - these are from
        // 2021/Mae
        new DriveProfileData(0.31, 2.74, 0.249),
        // TODO(mjh) Calibrate Sally's values for PID control - these are from 2021/Mae
        new PIDConfig(2.28, 0, 0),
        RobotSettings.DriveMotorInversion.Left,
        RobotSettings.GyroType.ADXRS450,
        0 // pigeonCanID
    );
  }

  /** Returns the robot settings for Mae (2021 robot). */
  private static RobotSettings getSettingsForMae() {
    return new RobotSettings(
        "Mae", // robotName
        Constants.TRACK_WIDTH_INCHES_MAE / Constants.INCHES_PER_METER,
        Constants.DRIVE_BASE_GEAR_RATIO_MAE,
        // Drive configuration constants (computed 01Mar2022 w/ SysId)
        new DriveProfileData(
            /* kS= */0.13895,
            /* kV= */1.3143,
            /* kA= */0.1935),
        // PID control constants (computed 01Mar2022 w/ SysId)
        new PIDConfig(0.0011379, 0, 0),
        RobotSettings.DriveMotorInversion.Left,
        RobotSettings.GyroType.ADXRS450,
        0 // pigeonCanID
    );
  }

  /** Returns the robot settings for Nike (2019 robot). */
  private static RobotSettings getSettingsForNike() {
    return new RobotSettings(
        "Nike", // robotName
        Constants.TRACK_WIDTH_INCHES_NIKE / Constants.INCHES_PER_METER,
        Constants.DRIVE_BASE_GEAR_RATIO_NIKE,
        // Drive configuration constants (computed 03Mar2022 w/ SysId)
        new DriveProfileData(
            /* kS= */0.14961,
            /* kV= */1.3717,
            /* kA= */0.1627),
        new PIDConfig(2.5682, 0, 0),
        RobotSettings.DriveMotorInversion.Left,
        RobotSettings.GyroType.Pigeon2,
        1 // pigeonCanID
    );
  }

  /** Returns the robot settings for use on a Romi. */
  private static RobotSettings getSettingsForRomi() {
    return new RobotSettings(
        "Romi", // robotName
        Constants.TRACK_WIDTH_METERS_ROMI,
        1, // TODO(mjh): Check gear ratio for the Romi
        // TODO(mjh): Recalibrate Romi's values for kS, kV, and kA (if SysId ever
        // supports this) - these are from 2021
        new DriveProfileData(1.25, 5.7, 0.0176),
        // TODO(mjh): Recalibrate Romi's values for PID control (if SysId ever
        // supports this) - these are from 2021
        new PIDConfig(0.00352, 0, 0),
        // Note: Romi docs indicate that it's the right motor that's inverted, but I run
        // the Romi in reverse because the USB camera is mounted on my upper deck that
        // way.
        RobotSettings.DriveMotorInversion.Left,
        RobotSettings.GyroType.Romi,
        0 // pigeonCanID
    );
  }

  /** Returns default settings (for the 2022 FRC robot). */
  private static RobotSettings getDefaultSettings() {
    return getSettingsForSally();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTankDriveCommand() {
    // Some simple bounds on driver inputs.
    SpeedModifier tankDriveDeadbandModifier = SpeedModifier.generateDeadbandSpeedModifier(Constants.Deadbands.DRIVING);
    SpeedModifier absoluteSpeedCaps = SpeedModifier.generateSpeedBounder(Constants.SpeedLimits.ABSOLUTE_LIMIT);

    // Mode signals for turtle & turbo.
    Supplier<Boolean> turtleSignalSupplier = () -> {
      return m_driverController.leftBumper().getAsBoolean();
    };
    Supplier<Boolean> turboSignalSupplier = () -> {
      return m_driverController.rightBumper().getAsBoolean();
    };

    // Build the speed modifier for normal / turtle / turbo support.
    SpeedModifier modeModifier = SpeedModifier.generateTurtleTurboSpeedModifier(
        Constants.SpeedLimits.MAX_SPEED_NORMAL,
        turtleSignalSupplier, Constants.SpeedLimits.MAX_SPEED_TURTLE,
        turboSignalSupplier, Constants.SpeedLimits.MAX_SPEED_TURBO);

    // Build the overall chain used to translate driver inputs into motor %ages.
    SpeedModifier compositeModifier = (double inputPercentage) -> absoluteSpeedCaps.adjustSpeed(
        modeModifier.adjustSpeed(tankDriveDeadbandModifier.adjustSpeed(inputPercentage)));

    // Generate the suppliers used to get "raw" speed signals for left and right.
    Supplier<Double> leftStickSpeedControl = () -> compositeModifier.adjustSpeed(m_driverController.getLeftY());
    Supplier<Double> rightStickSpeedControl = () -> compositeModifier.adjustSpeed(m_driverController.getRightY());
    m_switchModeHandler = new SwitchModeSpeedSupplier(leftStickSpeedControl, rightStickSpeedControl);

    // Get the (final) suppliers that will be polled for the left/right side speeds.
    Supplier<Double> leftSpeedControl = m_switchModeHandler.getLeftSpeedSupplier();
    Supplier<Double> rightSpeedControl = m_switchModeHandler.getRightSpeedSupplier();

    // Build the actual tank drive command.
    return new TankDrive(m_driveBase, leftSpeedControl, rightSpeedControl);
  }
}
