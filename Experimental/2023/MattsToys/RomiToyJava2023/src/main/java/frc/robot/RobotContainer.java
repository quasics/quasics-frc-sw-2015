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
import frc.robot.subsystems.PhotonVision;
import frc.robot.utils.RobotSettingsLibrary;
import frc.robot.utils.SpeedModifier;
import frc.robot.utils.SwitchModeSpeedSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final boolean USE_TANK_DRIVE = false;

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain(RobotSettingsLibrary.getSettingsForMattsRomi());
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  private final PhotonVision m_photonVision = new PhotonVision("photonvision", 0.10, // cameraHeightMeters
      0 // cameraPitchDegrees
  );

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
    m_drivetrain.setDefaultCommand(
        USE_TANK_DRIVE
            ? getTankDriveCommand()
            : getSplitArcadeDriveCommand());

    ///////////////////////////////////////////
    // Configure the various command bindings
    configureBindings();
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
    final double kP = 0.025;
    final double kI = 0.015;
    final double kD = 0.01;
    final int targetTag = 4;
    return new frc.robot.commands.DriveToAprilTag(m_drivetrain, m_photonVision, targetTag, 0.145, 0.10, 0.01, kP, kI,
        kD);
    // return new frc.robot.commands.TurnDegreesUsingPid(m_drivetrain, 90, 1, kP,
    // kI, kD);
    // return new frc.robot.commands.DriveDistance(0.25, 0.21991148575,
    // m_drivetrain);
    // return m_chooser.getSelected();
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

  /** Dead band modifier for driving controls. */
  final SpeedModifier m_drivingStickDeadbandModifier = SpeedModifier.generateDeadbandSpeedModifier(Deadbands.DRIVING);
  /** Absolute speed caps for driving controls. */
  final SpeedModifier m_absoluteSpeedCaps = SpeedModifier.generateSpeedBounder(SpeedLimits.ABSOLUTE_LIMIT);

  /** Signal supplier for "turtle mode" while driving. */
  final Supplier<Boolean> m_turtleSignalSupplier = () -> {
    return m_xboxController.getLeftBumper();
  };
  /** Signal supplier for "turbo mode" while driving. */
  final Supplier<Boolean> m_turboSignalSupplier = () -> {
    return m_xboxController.getRightBumper();
  };
  /** Signal supplier for "overdrive mode" while driving. */
  final Supplier<Boolean> m_overdriveSignalSupplier = () -> {
    return m_turtleSignalSupplier.get() && m_turboSignalSupplier.get();
  };

  /** The speed modifier for normal / turtle / turbo support. */
  final SpeedModifier m_modeSpeedModifier = SpeedModifier.generateTurtleTurboOverdriveSpeedModifier(
      SpeedLimits.MAX_SPEED_NORMAL,
      m_turtleSignalSupplier, SpeedLimits.MAX_SPEED_TURTLE,
      m_turboSignalSupplier, SpeedLimits.MAX_SPEED_TURBO,
      m_overdriveSignalSupplier, SpeedLimits.MAX_SPEED_OVERDRIVE);

  /**
   * Generates a command for running "split arcade drive", where the left stick
   * controls forward/backward speed, and the right stick controls rotational
   * speed.
   */
  public Command getSplitArcadeDriveCommand() {
    // Build the overall chain used to translate driver inputs into motor %ages.
    SpeedModifier compositeModifier = (double inputPercentage) -> m_absoluteSpeedCaps.adjustSpeed(
        m_modeSpeedModifier
            .adjustSpeed(m_drivingStickDeadbandModifier.adjustSpeed(inputPercentage)));

    // Cap the acceleration rate
    SpeedModifier forwardSlewRateModifier = SpeedModifier.generateSlewRateLimitModifier(SpeedLimits.MAX_SLEW_RATE);
    SpeedModifier rotationSlewRateModifier = SpeedModifier.generateSlewRateLimitModifier(SpeedLimits.MAX_SLEW_RATE);

    Supplier<Double> forwardSpeedSupplier = () -> {
      final double rawSpeed = m_xboxController.getLeftY();
      return -forwardSlewRateModifier.adjustSpeed(compositeModifier.adjustSpeed(rawSpeed));
    };
    Supplier<Double> rotationSpeedSupplier = () -> {
      final double rawSpeed = m_xboxController.getRightY();
      return -rotationSlewRateModifier.adjustSpeed(compositeModifier.adjustSpeed(rawSpeed));
    };

    return new ArcadeDrive(m_drivetrain, forwardSpeedSupplier, rotationSpeedSupplier);
  }

  /**
   * Generates a command for running "tank drive", where the left stick controls
   * forward/backward speed, and the right stick controls rotational speed.
   */
  public Command getTankDriveCommand() {
    // Mode signal for "drive straight" mode.
    //
    // Note: this might be better implemented as an alternate command, triggered
    // when the button is held down, and using PID control to prevent turning. But
    // this is a short-term solution, meant to demonstrate one approach.
    Supplier<Boolean> driveStraightSignalSupplier = () -> {
      return m_xboxController.getAButton();
    };

    // Cap the acceleration rate
    SpeedModifier leftSlewRateModifier = SpeedModifier.generateSlewRateLimitModifier(SpeedLimits.MAX_SLEW_RATE);
    SpeedModifier rightSlewRateModifier = SpeedModifier.generateSlewRateLimitModifier(SpeedLimits.MAX_SLEW_RATE);

    // Build the overall chain used to translate driver inputs into motor %ages.
    SpeedModifier compositeModifier = (double inputPercentage) -> m_absoluteSpeedCaps.adjustSpeed(
        m_modeSpeedModifier
            .adjustSpeed(m_drivingStickDeadbandModifier.adjustSpeed(inputPercentage)));

    // Generate the suppliers used to get "raw" speed signals for left and right.
    // On the GameSir and Xbox controllers, full forward = -1, full back = +1, so
    // we'll need to invert them to translate to what we're using in the drive base.
    //
    // Note that if we're in "drive straight lines" mode, we'll *only* read the left
    // stick. (Choice is arbitrary.)
    Supplier<Double> leftStickSupplier = () -> {
      double speed = m_xboxController.getLeftY();
      return -leftSlewRateModifier.adjustSpeed(compositeModifier.adjustSpeed(speed));
    };
    Supplier<Double> rightStickSupplier = () -> {
      final boolean inDriveStraightMode = driveStraightSignalSupplier.get();
      double speed = inDriveStraightMode ? m_xboxController.getLeftY() : m_xboxController.getRightY();
      return -rightSlewRateModifier.adjustSpeed(compositeModifier.adjustSpeed(speed));
    };

    // Get the (final) suppliers that will be polled for the left/right side
    // speeds.
    // m_switchModeHandler = new SwitchModeSpeedSupplier(leftStickSupplier,
    // rightStickSupplier);
    // Supplier<Double> leftSpeedControl =
    // m_switchModeHandler.getLeftSpeedSupplier();
    // Supplier<Double> rightSpeedControl =
    // m_switchModeHandler.getRightSpeedSupplier();
    final Supplier<Double> leftSpeedControl = leftStickSupplier;
    final Supplier<Double> rightSpeedControl = () -> {
      final boolean inDriveStraightMode = driveStraightSignalSupplier.get();
      return inDriveStraightMode ? leftStickSupplier.get() : rightStickSupplier.get();
    };

    // Build the actual tank drive command.
    return new TankDrive(m_drivetrain, leftSpeedControl, rightSpeedControl);
  }
}
