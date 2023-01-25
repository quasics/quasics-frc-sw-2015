// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import frc.robot.utils.RobotSettings;
import frc.robot.utils.SpeedModifier;
import frc.robot.utils.SwitchModeSpeedSupplier;
import frc.robot.utils.TrajectoryCommandGenerator.DriveProfileData;
import frc.robot.utils.TrajectoryCommandGenerator.PIDConfig;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain(getSettingsForRomi());
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes an Xbox Controller plugged into channnel 0
  private final XboxController m_xboxController = new XboxController(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * Used to manage "switch mode" (both providing the speed suppliers, and
   * managing current "switch mode" state).
   * 
   * This is set up by getTankDriveCommand().
   */
  private SwitchModeSpeedSupplier m_switchModeHandler;

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the
  // hardware "overlay" that is specified when launching the wpilib-ws server on
  // the Romi raspberry pi.
  //
  // By default, the following are available (listed in order from inside of the
  // board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    ///////////////////////////////////////////
    // Drive base setup

    // "Late initialization"
    m_drivetrain.finalizeSetup();

    // Default command for the subsystem.
    m_drivetrain.setDefaultCommand(getTankDriveCommand());

    ///////////////////////////////////////////
    // Configure the various command bindings
    configureBindings();
  }

  /** Returns the robot settings for use on a Romi. */
  private static RobotSettings getSettingsForRomi() {
    return new RobotSettings(
        "Romi", // robotName
        TRACK_WIDTH_METERS_ROMI,
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

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // Example of how to use the onboard IO
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("Button A Pressed"))
        .onFalse(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);

    // Add a command (triggered by the "Y" button) to trigger "switch mode" change.
    Trigger yButton = new JoystickButton(m_xboxController, XboxController.Button.kY.value);
    final Command changeDirectionCommand = runOnce(() -> {
      if (m_switchModeHandler != null) {
        System.out.println("Switching heading mode");
        m_switchModeHandler.toggleSwitchMode();
      }
    });
    yButton
        // .debounce(0.1) // Seems unreliable on Romi (too slow for feedback?)
        .onTrue(changeDirectionCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain,
        () -> -m_xboxController.getLeftY(),
        () -> m_xboxController.getLeftX());
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTankDriveCommand() {
    // Some simple bounds on driver inputs.
    SpeedModifier tankDriveDeadbandModifier = SpeedModifier.generateDeadbandSpeedModifier(Deadbands.DRIVING);
    SpeedModifier absoluteSpeedCaps = SpeedModifier.generateSpeedBounder(SpeedLimits.ABSOLUTE_LIMIT);

    // Matt's Romi has additional hardware (upper deck, camera), which makes it
    // easier to work with if we treat the front end as the back (since the camera
    // is easiest to mount pointing towards the nominal rear).
    SpeedModifier flippedRomiModifier = SpeedModifier.generateSpeedScaler(-1);

    // Mode signals for turtle & turbo.
    Supplier<Boolean> turtleSignalSupplier = () -> {
      return m_xboxController.getLeftBumper();
    };
    Supplier<Boolean> turboSignalSupplier = () -> {
      return m_xboxController.getRightBumper();
    };

    // Cap the acceleration rate
    SpeedModifier leftSlewRateModifier = SpeedModifier.generateSlewRateLimitModifier(SpeedLimits.MAX_SLEW_RATE);
    SpeedModifier rightSlewRateModifier = SpeedModifier.generateSlewRateLimitModifier(SpeedLimits.MAX_SLEW_RATE);

    // Build the speed modifier for normal / turtle / turbo support.
    SpeedModifier modeModifier = SpeedModifier.generateTurtleTurboSpeedModifier(
        SpeedLimits.MAX_SPEED_NORMAL,
        turtleSignalSupplier, SpeedLimits.MAX_SPEED_TURTLE,
        turboSignalSupplier, SpeedLimits.MAX_SPEED_TURBO);

    // Build the overall chain used to translate driver inputs into motor %ages.
    SpeedModifier compositeModifier = (double inputPercentage) -> absoluteSpeedCaps.adjustSpeed(
        modeModifier
            .adjustSpeed(tankDriveDeadbandModifier.adjustSpeed(flippedRomiModifier.adjustSpeed(inputPercentage))));

    // Generate the suppliers used to get "raw" speed signals for left and right.
    Supplier<Double> leftStickSpeedControl = () -> {
      double speed = m_xboxController.getRawAxis(1);
      return leftSlewRateModifier.adjustSpeed(compositeModifier.adjustSpeed(speed));
    };
    Supplier<Double> rightStickSpeedControl = () -> {
      double speed = m_xboxController.getRawAxis(5);
      return rightSlewRateModifier.adjustSpeed(compositeModifier.adjustSpeed(speed));
    };

    // Get the (final) suppliers that will be polled for the left/right side
    // speeds.
    // m_switchModeHandler = new SwitchModeSpeedSupplier(leftStickSpeedControl, rightStickSpeedControl);
    // Supplier<Double> leftSpeedControl = m_switchModeHandler.getLeftSpeedSupplier();
    // Supplier<Double> rightSpeedControl = m_switchModeHandler.getRightSpeedSupplier();

    // Build the actual tank drive command.
    return new TankDrive(m_drivetrain, leftStickSpeedControl, rightStickSpeedControl);
  }
}
