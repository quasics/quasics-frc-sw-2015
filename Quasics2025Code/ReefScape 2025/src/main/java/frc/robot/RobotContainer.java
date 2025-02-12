// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.MoveArmPivot;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunKraken;
import frc.robot.commands.RunKrakenForTime;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ArmRoller;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivebase.IDrivebase;
import frc.robot.subsystems.drivebase.RealDrivebase;
import frc.robot.subsystems.drivebase.SimulationDrivebase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.RobotSettings;
import java.util.function.Supplier;

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
  // The robot's subsystems and commands are defined here...
  private boolean m_switchDrive = false;
  private final IDrivebase m_drivebase;
  private final ArmPivot m_armPivot = new ArmPivot();
  private final ArmRoller m_armRoller = new ArmRoller();
  private final Elevator m_elevator = new Elevator();
  private final Climbers m_climbers = new Climbers();
  private final Vision m_vision = new Vision();

  private static final RobotSettings.Robot SETTINGS_FOR_REAL_MODE = RobotSettings.Robot.Sally;

  Supplier<Double> m_tankDriveLeftStick;
  Supplier<Double> m_tankDriveRightStick;
  Supplier<Double> m_arcadeDriveLeftStick;
  Supplier<Double> m_arcadeDriveRightStick;

  private final SlewRateLimiter m_leftSpeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rightSpeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_arcadeSpeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(1);

  private final Joystick m_driverController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);
  private final Joystick m_operatorController = new Joystick(Constants.DriveTeam.OPERATOR_JOYSTICK_ID);
  private final double DEADBAND_CONSTANT = 0.08;

  Trigger switchDriveTrigger;

  SendableChooser<String> m_autonomousOperations = new SendableChooser<String>();
  SendableChooser<String> m_positionOptions = new SendableChooser<String>();

  public static RobotSettings.Robot getRobotSettings() {
    if (RobotBase.isReal()) {
      return SETTINGS_FOR_REAL_MODE;
    } else {
      return RobotSettings.Robot.Simulator;
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivebase = setupDriveBase();
    // Configure the trigger bindings
    configureBindings();
    addButtonsToSmartDashboard();
    addOverallSelectorToSmartDashboard();
    addAutonomousStartingPositionsToSmartDashboard();
    ConfigureDriverButtons();
    ConfigureOperatorButtons();
  }

  private IDrivebase setupDriveBase() {
    IDrivebase drivebase = null;
    if (Robot.isReal()) {
      drivebase = new RealDrivebase(getRobotSettings());

      m_arcadeDriveLeftStick = () -> -m_driverController.getRawAxis(Constants.LogitechGamePad.LeftYAxis);
      m_arcadeDriveRightStick = () -> -m_driverController.getRawAxis(Constants.LogitechGamePad.RightXAxis);
    } else {
      m_arcadeDriveLeftStick = () -> -m_driverController.getRawAxis(0);
      m_arcadeDriveRightStick = () -> -m_driverController.getRawAxis(1);
      drivebase = new SimulationDrivebase(RobotSettings.Robot.Simulator);
    }
    return drivebase;
  }

  private double getDriverAxis(int controllerCode) {
    double axis = m_driverController.getRawAxis(controllerCode);
    // dead band enforcer
    return (Math.abs(axis) > DEADBAND_CONSTANT) ? axis : 0;
  }

  private void addButtonsToSmartDashboard() {
    addSysIdButtonsToSmartDashboard();
    addTestButtonsToSmartDashboard();
  }

  private void addTestButtonsToSmartDashboard() {
    SmartDashboard.putData(
        "Reset odometry", new InstantCommand(() -> m_drivebase.resetOdometry(new Pose2d())));

    SmartDashboard.putData("Arm Pivot Up", m_armPivot.setArmPivotUp());
    SmartDashboard.putData("Arm Pivot Down", m_armPivot.setArmPivotDown());

    SmartDashboard.putData("Arm Pivot 0", m_armPivot.setArmPivotUp());
    SmartDashboard.putData("Arm Pivot 90", m_armPivot.setArmPivotDown());

    SmartDashboard.putData(
        "Reset elevator encoders", new InstantCommand(() -> m_elevator.resetEncoders()));

    // SmartDashboard.putData("Drive 3m/s sim", new DriveForTime(m_drivebase,
    // Seconds.of(3), new
    // ChassisSpeeds(MetersPerSecond.of(3), MetersPerSecond.of(0),
    // RadiansPerSecond.of(0))));
  }

  private void addSysIdButtonsToSmartDashboard() {
    /*
     * SmartDashboard.putData(
     * "Quasistatic Forward",
     * m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * SmartDashboard.putData(
     * "Quasistatic Reverse",
     * m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * SmartDashboard.putData(
     * "Dynamic Forward",
     * m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * SmartDashboard.putData(
     * "Dynamic Reverse",
     * m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));
     */
  }

  private void addOverallSelectorToSmartDashboard() {
    m_autonomousOperations.setDefaultOption(Constants.AutonomousSelectedOperation.doNothing,
        Constants.AutonomousSelectedOperation.doNothing);
    m_autonomousOperations.addOption(
        Constants.AutonomousSelectedOperation.GTFO, Constants.AutonomousSelectedOperation.GTFO);

    SmartDashboard.putData("Overall operation", m_autonomousOperations);
  }

  private void addAutonomousStartingPositionsToSmartDashboard() {
    m_positionOptions.setDefaultOption(Constants.AutonomousStartingPositions.examplePosition,
        Constants.AutonomousStartingPositions.examplePosition);

    SmartDashboard.putData("Starting position", m_positionOptions);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_tankDriveLeftStick = () -> {
      double scalingFactor = getDriveSpeedScalingFactor();
      double axis = -getDriverAxis(Constants.LogitechGamePad.LeftYAxis);
      // needs testing to affirm direction is correct! if wrong just switch the -
      // signs
      if (m_switchDrive) {
        double joystickPercentage = -axis * scalingFactor;
        return m_leftSpeedLimiter.calculate(joystickPercentage);
      } else {
        double joystickPercentage = axis * scalingFactor;
        return m_leftSpeedLimiter.calculate(joystickPercentage);
      }
    };

    m_tankDriveRightStick = () -> {
      double scalingFactor = getDriveSpeedScalingFactor();
      double axis = -getDriverAxis(Constants.LogitechGamePad.RightYAxis);
      // needs testing also to affirm the direction is correct & switch signs if it is
      // wrong
      if (m_switchDrive) {
        double joystickPercentage = -axis * scalingFactor;
        return m_rightSpeedLimiter.calculate(joystickPercentage);
      } else {
        double joystickPercentage = axis * scalingFactor;
        return m_rightSpeedLimiter.calculate(joystickPercentage);
      }
    };

    m_arcadeDriveLeftStick = () -> {
      double scalingFactor = getDriveSpeedScalingFactor();
      double axis = -getDriverAxis(Constants.LogitechGamePad.LeftYAxis);
      if (m_switchDrive) {
        double joystickPercentage = axis * scalingFactor;
        return m_arcadeSpeedLimiter.calculate(joystickPercentage);
      } else {
        double joystickPercentage = -axis * scalingFactor;
        return m_arcadeSpeedLimiter.calculate(joystickPercentage);
      }
    };

    m_arcadeDriveRightStick = () -> {
      double scalingFactor = getDriveSpeedScalingFactor();

      double axis = -getDriverAxis(Constants.LogitechGamePad.RightXAxis);
      double joystickPercentage = axis * scalingFactor;
      return m_rotationLimiter.calculate(joystickPercentage);
    };

    switchDriveTrigger = new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.BButton))
        .onTrue(new InstantCommand(() -> {
          m_switchDrive = !m_switchDrive;
        }));

    m_drivebase.setDefaultCommand(
        new ArcadeDrive((m_drivebase), m_arcadeDriveLeftStick, m_arcadeDriveRightStick));
    // m_armRoller.setDefaultCommand(new RunKraken(m_armRoller, -0.1));
  }

  private Command intakeThenExtake() {
    return Commands.sequence(new RunKrakenForTime(m_armRoller, true, 0.2), new WaitCommand(0.1),
        new RunKraken(m_armRoller, 1.0));
  }

  private void ConfigureDriverButtons() {
    /*
     * Trigger extendClimber =
     * new Trigger(() ->
     * m_driverController.getRawButton(Constants.LogitechGamePad.YButton))
     * .whileTrue(new MoveClimbers(m_climbers, true));
     * Trigger retractClimber =
     * new Trigger(() ->
     * m_driverController.getRawButton(Constants.LogitechGamePad.AButton))
     * .whileTrue(new MoveClimbers(m_climbers, false));
     */

    // Register the triggers for various buttons on the controllers.
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.LeftTrigger))
        .whileTrue(new RunKraken(m_armRoller, -0.3));
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.RightTrigger))
        .whileTrue(intakeThenExtake());

    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.BackButton))
        .whileTrue(new RunElevator(m_elevator, 0.2)); // DOWN
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.StartButton))
        .whileTrue(new RunElevator(m_elevator, -0.2)); // UP

    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.YButton))
        .whileTrue(new MoveArmPivot(m_armPivot, 0.2, true)); // DOWN
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.AButton))
        .whileTrue(new MoveArmPivot(m_armPivot, 0.2, false)); // UP
  }

  private void ConfigureOperatorButtons() {
  }

  private double getDriveSpeedScalingFactor() {
    final boolean isTurbo = m_driverController.getRawButton(Constants.LogitechGamePad.RightShoulder);
    final boolean isTurtle = m_driverController.getRawButton(Constants.LogitechGamePad.LeftShoulder);

    if (isTurbo) {
      return Constants.RobotSpeedScaling.TURBO_MODE_SPEED_SCALING;
    } else if (isTurtle) {
      return Constants.RobotSpeedScaling.TURTLE_MODE_SPEED_SCALING;
    } else {
      return Constants.RobotSpeedScaling.NORMAL_MODE_SPEED_SCALING;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String autonomousOperation = m_autonomousOperations.getSelected();
    String positionOption = m_positionOptions.getSelected();

    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue); // default is
                                                                                                       // blue
    final boolean isBlue = alliance == DriverStation.Alliance.Blue;

    return Autos.getAutonomousCommand();
  }
}
