// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.MoveArmPivotToPosition;
import frc.robot.commands.MoveClimbersForTime;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.MoveElevatorToTargetPosition;
import frc.robot.commands.PulseKraken;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunKraken;
import frc.robot.commands.RunKrakenForTime;
import frc.robot.subsystems.ArmRoller;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.armPivot.ArmPivot;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.subsystems.drivebase.RealDrivebase;
import frc.robot.subsystems.drivebase.SimulationDrivebase;
import frc.robot.subsystems.elevator.AbstractElevator;
import frc.robot.subsystems.elevator.RealElevator;
import frc.robot.subsystems.elevator.SimulationElevator;
import frc.robot.utils.RobotSettings;
import frc.robot.utils.SysIdGenerator;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private boolean m_switchDrive = true;
  private final AbstractDrivebase m_drivebase = setupDriveBase();
  private final ArmPivot m_armPivot = new ArmPivot();
  private final ArmRoller m_armRoller = new ArmRoller();
  private final AbstractElevator m_elevator = setupElevator();
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

  private final AutoFactory m_autoFactory = new AutoFactory(
      m_drivebase::getPose, // A function that returns the current robot pose
      m_drivebase::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
      m_drivebase::followTrajectory, // The drive subsystem trajectory follower
      true, // flip path when on red side
      m_drivebase // The drive subsystem
  );

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

    // Configure the trigger bindings
    configureBindings();
    addButtonsToSmartDashboard();
    addOverallSelectorToSmartDashboard();
    addAutonomousStartingPositionsToSmartDashboard();
    ConfigureDriverButtons();
    ConfigureOperatorButtons();

  }

  private AbstractElevator setupElevator() {
    AbstractElevator elevator = null;
    if (Robot.isReal()) {
      elevator = new RealElevator();
    } else {
      elevator = new SimulationElevator();
    }
    return elevator;
  }

  private AbstractDrivebase setupDriveBase() {
    AbstractDrivebase drivebase = null;
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

    SmartDashboard.putData("Reset odometry test",
        new InstantCommand(
            () -> m_drivebase.resetOdometry(new Pose2d(Meters.of(3), Meters.of(3), new Rotation2d(Degrees.of(180))))));

    SmartDashboard.putData(
        "Arm Pivot Up", new InstantCommand(() -> m_armPivot.setArmPivotSpeed(-0.1)));
    SmartDashboard.putData(
        "Arm Pivot Down", new InstantCommand(() -> m_armPivot.setArmPivotSpeed(0.1)));

    SmartDashboard.putData("Arm Pivot PID Up", new MoveArmPivotToPosition(m_armPivot, Degrees.of(95)));
    SmartDashboard.putData("Arm Pivot PID Down", new MoveArmPivotToPosition(m_armPivot, Degrees.of(1)));
    SmartDashboard.putData("Arm Pivot 45", new MoveArmPivotToPosition(m_armPivot, Degrees.of(45)));

    SmartDashboard.putData("Stop arm pivot", new InstantCommand(() -> m_armPivot.stop()));

    SmartDashboard.putData(
        "Reset elevator encoders", new InstantCommand(() -> m_elevator.resetEncoders()));

    SmartDashboard.putData("Elevator Up", new InstantCommand(() -> m_elevator.setSpeed(-0.2)));
    SmartDashboard.putData("Elevator Down", new InstantCommand(() -> m_elevator.setSpeed(0.2)));

    SmartDashboard.putData(
        "Extend Climber 25% ", new MoveClimbersForTime(m_climbers, true, 0.25, .5));
    SmartDashboard.putData(
        "Extend Climber 50% ", new MoveClimbersForTime(m_climbers, true, 0.5, .25));
    SmartDashboard.putData(
        "Extend Climber 5% ", new MoveClimbersForTime(m_climbers, false, 0.5, 1));
    SmartDashboard.putData(
        "Extend Climber 10% ", new MoveClimbersForTime(m_climbers, false, .10, 1));
    SmartDashboard.putData(
        "Retract Climber 10% ", new MoveClimbersForTime(m_climbers, true, 0.10, .5));

    SmartDashboard.putData("Elevator to L2",
        new MoveElevatorToTargetPosition(m_elevator, AbstractElevator.TargetPosition.kL2));
    SmartDashboard.putData("Elevator to L1",
        new MoveElevatorToTargetPosition(m_elevator, AbstractElevator.TargetPosition.kL1));
    SmartDashboard.putData("Elevator to bottom",
        new MoveElevatorToTargetPosition(m_elevator, AbstractElevator.TargetPosition.kBottom));
    SmartDashboard.putData("Elevator to DC",
        new MoveElevatorToTargetPosition(m_elevator, AbstractElevator.TargetPosition.kDontCare));

    SmartDashboard.putData("Test path", testTrajectory("testPath"));

    // SmartDashboard.putData("Drive 3m/s sim", new DriveForTime(m_drivebase,
    // Seconds.of(3), new
    // ChassisSpeeds(MetersPerSecond.of(3), MetersPerSecond.of(0),
    // RadiansPerSecond.of(0))));
  }

  private Command testTrajectory(String name) {
    return Commands.sequence(
        m_autoFactory.resetOdometry(name),
        m_autoFactory.resetOdometry(name),
        m_autoFactory.trajectoryCmd(name));
  }

  private void addSysIdButtonsToSmartDashboard() {
    /*
     * old stuff? idk
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

    SmartDashboard.putData("SysID: Quasistatic(fwd)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.Mode.Linear, Direction.kForward));
    SmartDashboard.putData("SysID: Quasistatic(rev)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.Mode.Linear, Direction.kReverse));
    SmartDashboard.putData("SysID: Dynamic(fwd)",
        SysIdGenerator.sysIdDynamic(m_drivebase, SysIdGenerator.Mode.Linear, Direction.kForward));
    SmartDashboard.putData("SysID: Dynamic(rev)",
        SysIdGenerator.sysIdDynamic(m_drivebase, SysIdGenerator.Mode.Linear, Direction.kReverse));

    // SysId commands for rotational actions (used to calculate kA-angular), for use
    // in estimating the moment of inertia (MOI).
    // See: https://choreo.autos/usage/estimating-moi/
    SmartDashboard.putData("SysID(rot): Quasistatic(fwd)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.Mode.Rotating, Direction.kForward));
    SmartDashboard.putData("SysID(rot): Quasistatic(rev)",
        SysIdGenerator.sysIdQuasistatic(
            m_drivebase, SysIdGenerator.Mode.Rotating, Direction.kReverse));
    SmartDashboard.putData("SysID(rot): Dynamic(fwd)",
        SysIdGenerator.sysIdDynamic(m_drivebase, SysIdGenerator.Mode.Rotating, Direction.kForward));
    SmartDashboard.putData("SysID(rot): Dynamic(rev)",
        SysIdGenerator.sysIdDynamic(m_drivebase, SysIdGenerator.Mode.Rotating, Direction.kReverse));

  }

  private void addOverallSelectorToSmartDashboard() {
    m_autonomousOperations.setDefaultOption(Constants.AutonomousSelectedOperation.DO_NOTHING,
        Constants.AutonomousSelectedOperation.DO_NOTHING);
    m_autonomousOperations.addOption(
        Constants.AutonomousSelectedOperation.GTFO, Constants.AutonomousSelectedOperation.GTFO);
    m_autonomousOperations.addOption(
        Constants.AutonomousSelectedOperation.GO_TO_REEF, Constants.AutonomousSelectedOperation.GO_TO_REEF);
    m_autonomousOperations.addOption(
        Constants.AutonomousSelectedOperation.GO_TO_REEF_DR, Constants.AutonomousSelectedOperation.GO_TO_REEF_DR);
    m_autonomousOperations.addOption(Constants.AutonomousSelectedOperation.GRAB_ALGAE_FROM_REEF,
        Constants.AutonomousSelectedOperation.GRAB_ALGAE_FROM_REEF);
    m_autonomousOperations.addOption(
        Constants.AutonomousSelectedOperation.SCORE_ALGAE_REEF_BARGE,
        Constants.AutonomousSelectedOperation.SCORE_ALGAE_REEF_BARGE);
    m_autonomousOperations.addOption(
        Constants.AutonomousSelectedOperation.SCORE_ALGAE_REEF_PROCESSOR,
        Constants.AutonomousSelectedOperation.SCORE_ALGAE_REEF_PROCESSOR);

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
     * new Trigger(() ->
     * m_driverController.getRawButton(Constants.LogitechGamePad.YButton))
     * .whileTrue(new MoveClimbers(m_climbers, true));
     * new Trigger(() ->
     * m_driverController.getRawButton(Constants.LogitechGamePad.AButton))
     * .whileTrue(new MoveClimbers(m_climbers, false)); // Register the triggers for
     * various
     * // buttons on the controllers.
     */
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.LeftTrigger))
        .whileTrue(new RunKraken(m_armRoller, -0.3));
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.RightTrigger))
        .whileTrue(new PulseKraken(m_armRoller, -0.1, 0.2, 0.75));

    // Elevator controls
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.YButton))
        .whileTrue(new RunElevator(m_elevator, -0.85)); // UP
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.AButton))
        .whileTrue(new RunElevator(m_elevator, 0.75)); // DOWN
  }

  private void ConfigureOperatorButtons() {
    // Shooting
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kX.value))
        .whileTrue(intakeThenExtake());
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kB.value))
        .whileTrue(new RunKraken(m_armRoller, 0.5));

    // Arm Pivot Controls
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kRightBumper.value))
        .whileTrue(new MoveArmPivot(m_armPivot, 0.3));
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kLeftBumper.value))
        .whileTrue(new MoveArmPivot(m_armPivot, -0.3));
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kA.value))
        .whileTrue(new MoveArmPivotToPosition(m_armPivot, Degrees.of(95)));
  }

  private double getDriveSpeedScalingFactor() {
    final boolean isTurbo = m_driverController.getRawButton(Constants.LogitechGamePad.LeftShoulder);
    final boolean isTurtle = m_driverController.getRawButton(Constants.LogitechGamePad.RightShoulder);

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
    // String positionOption = m_positionOptions.getSelected();

    var positionOpt = DriverStation.getLocation();
    if (positionOpt.isEmpty()) {
      System.out.println("WARNING: Can't get position!");
      return Commands.none();
    }

    return Autos.getAutonomousCommand(m_autoFactory, m_drivebase, m_elevator, m_armPivot, autonomousOperation,
        positionOpt.getAsInt());
  }
}
