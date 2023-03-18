// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Deadbands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SpeedLimits;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.SimpleLighting;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnDegreesUsingPid;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.LightingInterface;
import frc.robot.utils.RobotSettings;
import frc.robot.utils.SpeedModifier;
import frc.robot.utils.SwitchModeSpeedSupplier;
import java.util.List;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // TODO: Modify this to try to load current settings from filesystem.
  private final Drivebase m_driveBase = new Drivebase(ROBOT_SETTINGS);

  private final LightingInterface m_lighting = RobotSettings.isValidPwmPort(ROBOT_SETTINGS.ledPort)
      ? new Lighting(LightingValues.LED_STRIP_PWM_PORT, LightingValues.LED_STRIP_LENGTH)
      : new LightingInterface.MockLighting();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public enum AutoModeOperation {
    eDoNothing,
    eMoveBack;

    public String getName() {
      String s = toString().replaceAll("^e", ""); // strip leading "e"
      return s.replaceAll("(.)([A-Z])", "$1 $2"); // insert spaces before caps (other than first)
    }
  }

  SendableChooser<RobotPosition> teamPositionChooser = new SendableChooser<RobotPosition>();
  SendableChooser<AutoModeOperation> commandChooser = new SendableChooser<AutoModeOperation>();

  /**
   * Used to manage "switch mode" (both providing the speed suppliers, and
   * managing current "switch
   * mode" state).
   *
   * <p>
   * This is set up by getTankDriveCommand().
   */
  private SwitchModeSpeedSupplier m_switchModeHandler;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //////////////////////////////////////
    // Drive base setup

    // "Late initialization"
    m_driveBase.finalizeSetup();

    // Default command for the subsystem.
    m_driveBase.setDefaultCommand(getTankDriveCommand());

    //////////////////////////////////////
    // Lighting setup
    ((Subsystem) m_lighting).setDefaultCommand(new SimpleLighting(m_lighting));

    //////////////////////////////////////
    // Configure the trigger bindings
    configureBindings();

    //////////////////////////////////////
    // Set up the choices of things to do during auto mode.
    setupAutoSelection();
  }

  private void configureTestCommands() {
    SmartDashboard.putData("Turn +45 degrees", new TurnDegreesUsingPid(m_driveBase, +45));
    SmartDashboard.putData("Turn -90 degrees", new TurnDegreesUsingPid(m_driveBase, -90));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox} and
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers, or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Add a command (triggered by the "Y" button on the driver controller) to
    // trigger "switch mode" change.
    Trigger yButton = m_driverController.y();
    final Command changeDirectionCommand = runOnce(
        () -> {
          if (m_switchModeHandler != null) {
            System.out.println("Switching heading mode");
            m_switchModeHandler.toggleSwitchMode();
          }
        });
    yButton.debounce(0.1).onTrue(changeDirectionCommand);

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`.
    //
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    //
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  static final List<String> operations = List.of("Do nothing");

  private void setupAutoSelection() {
    for (RobotPosition pos : RobotPosition.values()) {
      teamPositionChooser.addOption(pos.toString(), pos);
    }
    SmartDashboard.putData("Team position", teamPositionChooser);

    for (AutoModeOperation op : AutoModeOperation.values()) {
      commandChooser.addOption(op.getName(), op);
    }
    SmartDashboard.putData("Auto operation", commandChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    final RobotPosition pos = teamPositionChooser.getSelected();
    final AutoModeOperation op = commandChooser.getSelected();

    // Sanity check
    if (pos == null || op == null) {
      return new PrintCommand("*** Error: Can't detect position and/or op for auto mode!");
    }

    // Build the command/sequence, based on operation and position.
    switch (op) {
      case eDoNothing:
        return new PrintCommand("Auto mode: Doing nothing, as instructed");
      case eMoveBack:
        return new SequentialCommandGroup(
            new PrintCommand("Auto mode: driving backward from position " + pos),
            new DriveDistance(m_driveBase, -0.5, pos.distanceToCommunityLineInMeters()));
    }

    // Fallback case.
    return new PrintCommand("*** Error: Couldn't generate command for auto mode!");
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

    // Mode signals for turtle & turbo.
    Supplier<Boolean> turtleSignalSupplier = () -> {
      return m_driverController.leftBumper().getAsBoolean();
    };
    Supplier<Boolean> turboSignalSupplier = () -> {
      return m_driverController.rightBumper().getAsBoolean();
    };

    // Build the speed modifier for normal / turtle / turbo support.
    SpeedModifier modeModifier = SpeedModifier.generateTurtleTurboSpeedModifier(
        SpeedLimits.MAX_SPEED_NORMAL,
        turtleSignalSupplier,
        SpeedLimits.MAX_SPEED_TURTLE,
        turboSignalSupplier,
        SpeedLimits.MAX_SPEED_TURBO);

    // Cap the acceleration rate
    SpeedModifier slewRateModifier = SpeedModifier.generateSlewRateLimitModifier(SpeedLimits.MAX_SLEW_RATE);

    // Build the overall chain used to translate driver inputs into motor %ages.
    SpeedModifier compositeModifier = (double inputPercentage) -> absoluteSpeedCaps.adjustSpeed(
        slewRateModifier.adjustSpeed(
            modeModifier.adjustSpeed(
                tankDriveDeadbandModifier.adjustSpeed(inputPercentage))));

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
