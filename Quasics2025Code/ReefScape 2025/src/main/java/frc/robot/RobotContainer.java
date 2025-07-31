// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutonomousSelectedOperation;
import frc.robot.Constants.AutonomousStartingPositions;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmPivotToPositionOnController;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTeamCandle;
import frc.robot.commands.ElevatorToPositionOnController;
import frc.robot.commands.MoveArmPivot;
import frc.robot.commands.MoveArmPivotAndElevatorToPosition;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.PulseKraken;
import frc.robot.commands.RunKraken;
import frc.robot.commands.RunKrakenForTime;
import frc.robot.subsystems.ArmPivot;
import frc.robot.subsystems.ArmRoller;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.candle.AbstractCandle;
import frc.robot.subsystems.candle.Candle;
import frc.robot.subsystems.candle.SimCandle;
import frc.robot.subsystems.drivebase.AbstractDrivebase;
import frc.robot.subsystems.drivebase.RealDrivebase;
import frc.robot.subsystems.drivebase.SimulationDrivebase;
import frc.robot.subsystems.elevator.AbstractElevator;
import frc.robot.subsystems.elevator.AbstractElevator.TargetPosition;
import frc.robot.subsystems.elevator.RealElevator;
import frc.robot.subsystems.elevator.SimulationElevator;
import frc.robot.utils.RobotSettings;
import frc.robot.utils.SysIdGenerator;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final static boolean ENABLE_CANDLE = true;

  // The robot's subsystems and commands are defined here...
  private boolean m_switchDrive = true;
  private final AbstractDrivebase m_drivebase = setupDriveBase();
  private final ArmPivot m_armPivot = new ArmPivot();
  private final ArmRoller m_armRoller = new ArmRoller();
  private final AbstractElevator m_elevator = setupElevator();
  @SuppressWarnings("unused")
  private final Vision m_vision = new Vision(m_drivebase::getPose);
  private final AbstractCandle m_candle = allocateCandle();

  private static AbstractCandle allocateCandle() {
    if (!ENABLE_CANDLE) {
      return null;
    } else if (RobotBase.isSimulation()) {
      return new SimCandle();
    }
    return new Candle();
  }

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

  private final AutoFactory m_autoFactory = new AutoFactory(m_drivebase::getPose, // A function that returns the current
                                                                                  // robot pose
      m_drivebase::resetOdometry, // A function that resets the current robot pose to the
                                  // provided Pose2d
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

    if (m_candle != null) {
      // Default to candle being green at startup.
      m_candle.setColor(Candle.GREEN);
    }
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

      m_arcadeDriveLeftStick = () -> -m_driverController.getRawAxis(Constants.LogitechDualshock.LeftYAxis);
      m_arcadeDriveRightStick = () -> -m_driverController.getRawAxis(Constants.LogitechDualshock.RightXAxis);
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
    // addSysIdButtonsToSmartDashboard();
    addTestButtonsToSmartDashboard();
  }

  private void addTestButtonsToSmartDashboard() {
    /*
     * SmartDashboard.putData(
     * "Reset odometry", new InstantCommand(() -> m_drivebase.resetOdometry(new
     * Pose2d())));
     *
     * SmartDashboard.putData("Reset odometry test",
     * new InstantCommand(
     * () -> m_drivebase.resetOdometry(new Pose2d(Meters.of(3), Meters.of(3), new
     * Rotation2d(Degrees.of(180))))));
     *
     * SmartDashboard.putData(
     * "Arm Pivot Up", new InstantCommand(() -> m_armPivot.setArmPivotSpeed(-0.1)));
     * SmartDashboard.putData(
     * "Arm Pivot Down", new InstantCommand(() ->
     * m_armPivot.setArmPivotSpeed(0.1)));
     *
     * SmartDashboard.putData("Arm Pivot PID Up", new
     * MoveArmPivotToPosition(m_armPivot, Degrees.of(95)));
     * SmartDashboard.putData("Arm Pivot PID Down", new
     * MoveArmPivotToPosition(m_armPivot, Degrees.of(1)));
     * SmartDashboard.putData("Arm Pivot 45", new MoveArmPivotToPosition(m_armPivot,
     * Degrees.of(45)));
     *
     * SmartDashboard.putData("Stop arm pivot", new InstantCommand(() ->
     * m_armPivot.stop()));
     */
    SmartDashboard.putData(
        "Reset elevator encoders", new InstantCommand(() -> m_elevator.resetEncoders()));

    SmartDashboard.putData("Elevator to top",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.kTop));
    SmartDashboard.putData("Elevator to L2",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.kL2));
    SmartDashboard.putData("Elevator to L1",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.kL1));
    SmartDashboard.putData("Elevator to bottom",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.kBottom));
    SmartDashboard.putData("Elevator to DC",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.kDontCare));

    SmartDashboard.putData("Test path", testTrajectory("testPath"));

    // SmartDashboard.putData("Drive 3m/s sim", new DriveForTime(m_drivebase,
    // Seconds.of(3), new
    // ChassisSpeeds(MetersPerSecond.of(3), MetersPerSecond.of(0),
    // RadiansPerSecond.of(0))));
  }

  private Command testTrajectory(String name) {
    return Commands.sequence(m_autoFactory.resetOdometry(name), m_autoFactory.resetOdometry(name),
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
    m_autonomousOperations.setDefaultOption(
        AutonomousSelectedOperation.DO_NOTHING, AutonomousSelectedOperation.DO_NOTHING);
    m_autonomousOperations.addOption(
        AutonomousSelectedOperation.GTFO, AutonomousSelectedOperation.GTFO);
    m_autonomousOperations.addOption(
        AutonomousSelectedOperation.GO_TO_REEF, AutonomousSelectedOperation.GO_TO_REEF);
    m_autonomousOperations.addOption(
        AutonomousSelectedOperation.GO_TO_REEF_DR, AutonomousSelectedOperation.GO_TO_REEF_DR);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.GRAB_ALGAE_FROM_REEF,
        AutonomousSelectedOperation.GRAB_ALGAE_FROM_REEF);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.SCORE_CORAL_IN_REEF,
        AutonomousSelectedOperation.SCORE_CORAL_IN_REEF);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.SCORE_ALGAE_REEF_BARGE,
        AutonomousSelectedOperation.SCORE_ALGAE_REEF_BARGE);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.SCORE_ALGAE_REEF_PROCESSOR,
        AutonomousSelectedOperation.SCORE_ALGAE_REEF_PROCESSOR);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.SCORE_CORAL_GRAB_ALGAE,
        AutonomousSelectedOperation.SCORE_CORAL_GRAB_ALGAE);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.SCORE_CORAL_SCORE_BARGE,
        AutonomousSelectedOperation.SCORE_CORAL_SCORE_BARGE);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.SCORE_CORAL_SCORE_PROCESSOR,
        AutonomousSelectedOperation.SCORE_CORAL_SCORE_PROCESSOR);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.GRAB_ALGAE_FROM_FIELD,
        AutonomousSelectedOperation.GRAB_ALGAE_FROM_FIELD);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.SCORE_ALGAE_FIELD_BARGE,
        AutonomousSelectedOperation.SCORE_ALGAE_FIELD_BARGE);
    m_autonomousOperations.addOption(AutonomousSelectedOperation.SCORE_ALGAE_FIELD_PROCESSOR,
        AutonomousSelectedOperation.SCORE_ALGAE_FIELD_PROCESSOR);

    SmartDashboard.putData("Overall operation", m_autonomousOperations);
  }

  private void addAutonomousStartingPositionsToSmartDashboard() {
    m_positionOptions.setDefaultOption(
        AutonomousStartingPositions.VERY_TOP, AutonomousStartingPositions.VERY_TOP);
    m_positionOptions.addOption(AutonomousStartingPositions.TOP, AutonomousStartingPositions.TOP);
    m_positionOptions.addOption(
        AutonomousStartingPositions.MIDDLE, AutonomousStartingPositions.MIDDLE);
    m_positionOptions.addOption(
        AutonomousStartingPositions.BOTTOM, AutonomousStartingPositions.BOTTOM);
    m_positionOptions.addOption(
        AutonomousStartingPositions.VERY_BOTTOM, AutonomousStartingPositions.VERY_BOTTOM);

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
      double axis = -getDriverAxis(Constants.LogitechDualshock.LeftYAxis);
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
      double axis = -getDriverAxis(Constants.LogitechDualshock.RightYAxis);
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
      double axis = -getDriverAxis(Constants.LogitechDualshock.LeftYAxis);
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

      double axis = -getDriverAxis(Constants.LogitechDualshock.RightXAxis);
      double joystickPercentage = axis * scalingFactor;
      return m_rotationLimiter.calculate(joystickPercentage);
    };

    switchDriveTrigger = new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.BButton))
        .onTrue(new InstantCommand(() -> {
          m_switchDrive = !m_switchDrive;
        }));

    if (m_candle != null) {
      ((SubsystemBase) m_candle).setDefaultCommand(new DriveTeamCandle(m_candle, m_drivebase));
    }
    m_drivebase.setDefaultCommand(
        new ArcadeDrive((m_drivebase), m_arcadeDriveLeftStick, m_arcadeDriveRightStick));
    // m_armRoller.setDefaultCommand(new RunKraken(m_armRoller, -0.1));
    m_armRoller.setDefaultCommand(new PulseKraken(m_armRoller, -0.1, 0.2, 0.75));
  }

  private Command intakeThenExtake() {
    return Commands.sequence(new RunKrakenForTime(m_armRoller, -0.3, 0.2), new WaitCommand(0.1),
        new RunKrakenForTime(m_armRoller, 1.0, 0.5));
  }

  private Command shootWithElevator() {
    return Commands.parallel(
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.kTop),
        Commands.sequence(new WaitCommand(1.5), intakeThenExtake()));
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
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.LeftTrigger))
        .whileTrue(new RunKraken(m_armRoller, -0.5));
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.RightTrigger))
        .whileTrue(new PulseKraken(m_armRoller, -0.1, 0.2, 0.75));

    // Elevator controls
    /*
     * new Trigger(() ->
     * m_driverController.getRawButton(Constants.LogitechDualshock.YButton))
     * .whileTrue(new RunElevator(m_elevator, -0.3)); // UP
     * new Trigger(() ->
     * m_driverController.getRawButton(Constants.LogitechDualshock.AButton))
     * .whileTrue(new RunElevator(m_elevator, 0.3)); // DOWN
     */
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.YButton))
        .whileTrue(new ElevatorToPositionOnController(m_elevator, TargetPosition.kTop));
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.AButton))
        .whileTrue(new ElevatorToPositionOnController(m_elevator, TargetPosition.kBottom));

    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.StartButton))
        .whileTrue(new MoveArmPivotAndElevatorToPosition(
            m_armPivot, m_elevator, Degrees.of(95), TargetPosition.kTop));
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.BackButton))
        .whileTrue(new MoveArmPivotAndElevatorToPosition(
            m_armPivot, m_elevator, Degrees.of(0), TargetPosition.kBottom));
  }

  private void ConfigureOperatorButtons() {
    // Shooting
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kX.value))
        .whileTrue(intakeThenExtake());
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kB.value))
        .whileTrue(new RunKraken(m_armRoller, 0.5));

    // Arm Pivot Controls
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kRightBumper.value))
        .whileTrue(new MoveArmPivot(m_armPivot, 0.35));
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kLeftBumper.value))
        .whileTrue(new MoveArmPivot(m_armPivot, -0.35));

    // PID controls (both armpivot and elevator)
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kA.value))
        .whileTrue(new MoveArmPivotAndElevatorToPosition(
            m_armPivot, m_elevator, Degrees.of(33), TargetPosition.kL1));
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kY.value))
        .whileTrue(new MoveArmPivotAndElevatorToPosition(
            m_armPivot, m_elevator, Degrees.of(33), TargetPosition.kL2));
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kBack.value))
        .whileTrue(new ArmPivotToPositionOnController(m_armPivot, Degrees.of(95)));

    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kLeftStick.value))
        .whileTrue(new ArmPivotToPositionOnController(m_armPivot, Degrees.of(25)));
    new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kRightStick.value))
        .whileTrue(new ArmPivotToPositionOnController(m_armPivot, Degrees.of(21)));
  }

  private double getDriveSpeedScalingFactor() {
    final boolean isTurbo = m_driverController.getRawButton(Constants.LogitechDualshock.LeftShoulder);
    final boolean isTurtle = m_driverController.getRawButton(Constants.LogitechDualshock.RightShoulder);

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

    return Autos.getAutonomousCommand(m_autoFactory, m_drivebase, m_elevator, m_armPivot,
        m_armRoller, autonomousOperation, positionOption);
  }
}
