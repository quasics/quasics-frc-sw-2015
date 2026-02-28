// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveteamConstants;
import frc.robot.Constants.PwmPortIds;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.LinearSpeedCommand;
import frc.robot.commands.PivotHoodToPosition;
import frc.robot.commands.RainbowLighting;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntakeExtension;
import frc.robot.commands.RunIntakeRollers;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterForTime;
import frc.robot.commands.RunShooterPID;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IShooterHood;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IIndexer;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.subsystems.real.Lighting;
import frc.robot.subsystems.real.LightingBuffer;
import frc.robot.subsystems.real.NovaDriveBase;
import frc.robot.subsystems.real.RealClimber;
import frc.robot.subsystems.real.RealIndexer;
import frc.robot.subsystems.real.RealIntake;
import frc.robot.subsystems.real.RealShooter;
import frc.robot.subsystems.real.RealShooterHood;
import frc.robot.subsystems.real.SparkDriveBase;
import frc.robot.subsystems.real.Vision;
import frc.robot.subsystems.simulated.SimulatedVision;
import frc.robot.subsystems.simulated.SimulationDrivebase;

import static edu.wpi.first.units.Units.RPM;

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
  private static final int SIDE_LIGHTING_LENGTH = Constants.LIGHTING_TOTAL_LENGTH / 2;

  /**
   * Names of Quasics robots on which this code might be executed.
   * 
   * Note that this is effectively providing a limited mechanism for per-robot
   * configuration.
   */
  enum RobotName {
    Simulated,
    Lizzie,
    Sally
  }

  /**
   * Identifies the default robot that we'll assume is in use when we're not under
   * simulation.
   * 
   * Note that this is effectively providing a limited mechanism for per-robot
   * configuration.
   */
  private static final RobotName DEFAULT_ROBOT_NAME = RobotName.Lizzie;

  /**
   * The robot name we'll actually use while executing (which will account for
   * simulation).
   * 
   * Note that this is effectively providing a limited mechanism for per-robot
   * configuration.
   */
  private static final RobotName ROBOT_NAME = Robot.isReal() ? DEFAULT_ROBOT_NAME : RobotName.Simulated;

  private final IIntake m_intake = (ROBOT_NAME == RobotName.Lizzie) ? new RealIntake() : new IIntake.NullIntake();
  private final IIndexer m_indexer = (ROBOT_NAME == RobotName.Lizzie) ? new RealIndexer() : new IIndexer.NullIndexer();
  private final IShooterHood m_hood = (ROBOT_NAME == RobotName.Lizzie) ? new RealShooterHood()
      : new IShooterHood.NullShooterHood();
  private final IShooter m_shooter = (ROBOT_NAME == RobotName.Lizzie) ? new RealShooter()
      : new IShooter.NullShooter();
  private final IClimber m_climber = (ROBOT_NAME == RobotName.Lizzie) ? new RealClimber() : new IClimber.NullClimber();

  // The robot's subsystems and commands are defined here...
  private final IDrivebase m_drivebase = switch (ROBOT_NAME) {
    case Sally -> new SparkDriveBase();
    case Simulated -> new SimulationDrivebase();
    case Lizzie -> new NovaDriveBase();
  };
  private final IVision m_vision = (Robot.isReal()) ? new Vision() : new SimulatedVision();

  private final ILighting m_primaryLighting;
  private final ILighting m_leftSideLighting;
  private final ILighting m_rightSideLighting;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController = new Joystick(DriveteamConstants.DRIVER_JOYSTICK_ID);
  private final Joystick m_operatorController = new Joystick(DriveteamConstants.OPERATOR_JOYSTICK_ID);

  private final double DEADBAND_CONSTANT = 0.08;
  private final SlewRateLimiter m_speedSlewRateLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotSlewRateLimiter = new SlewRateLimiter(1);

  private boolean m_switchDrive = false;

  Supplier<Double> m_arcadeDriveLeftStick;
  Supplier<Double> m_arcadeDriveRightStick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    m_primaryLighting = allocatePrimaryLighting();
    m_leftSideLighting = allocateSideLighting(true);
    m_rightSideLighting = allocateSideLighting(false);

    m_leftSideLighting.setAsDefaultCommand(new RainbowLighting(m_leftSideLighting));
    m_rightSideLighting.setAsDefaultCommand(new RainbowLighting(m_rightSideLighting));

    // Connect cross-subsystem suppliers (so that the systems don't know about
    // each other directly)
    addButtonsToSmartDashboard();
    addSysIdButtonsToSmartDashboard();

    m_vision.setReferencePositionSupplier(() -> {
      if (m_drivebase != null) {
        return m_drivebase.getEstimatedPose();
      } else {
        return null;
      }
    });

    m_drivebase.setReferencePositionSupplier(() -> {
      if (m_vision != null) {
        return m_vision.getVisionLatestPose();
      } else {
        return null;
      }
    });

    // Configure the trigger bindings
    configureBindings();
    configureDriverButtons();
  }

  private ILighting allocatePrimaryLighting() {
    return new Lighting(
        PwmPortIds.LIGHTING_ID,
        Constants.LIGHTING_TOTAL_LENGTH,
        List.of(SIDE_LIGHTING_LENGTH, SIDE_LIGHTING_LENGTH));
  }

  private ILighting allocateSideLighting(boolean isLeftSide) {
    if (!(m_primaryLighting instanceof Lighting)) {
      // Won't be able to pull subviews for side-specific lighting.
      return new ILighting.NullLighting();
    }

    final Lighting realLighting = (Lighting) m_primaryLighting;
    final int targetIndex = (isLeftSide ? 0 : 1);
    if (realLighting.getSubViews().size() <= targetIndex) {
      // Can't get a subsystem that's mapped for that side
      return new ILighting.NullLighting();
    }

    return new LightingBuffer(realLighting.getSubViews().get(targetIndex), isLeftSide);
  }

  private void addButtonsToSmartDashboard() {
    if (m_intake != null) {
      SmartDashboard.putData("Run Intake Rollers", new RunIntakeRollers(m_intake, 0.1, true));
      SmartDashboard.putData("Run Indexer", new RunIndexer(m_indexer, 0.1, true));
      // SmartDashboard.putData("Extend Intake", new InstantCommand(() ->
      // m_intake.setExtensionSpeed(0.5)));
      // SmartDashboard.putData("Extend Intake", new InstantCommand(() ->
      // m_intake.setExtensionSpeed(-0.5)));
    }

    if (m_shooter != null) {
      SmartDashboard.putData("Run Flywheel @ 1200 RPM, Kicker @ 12.5% speed",
          new RunShooterPID(m_shooter, RPM.of(1200), .125));
      SmartDashboard.putData("Run Flywheel @ 3700 RPM, Kicker @ 38.7% speed",
          new RunShooterPID(m_shooter, RPM.of(3700), .387));
      SmartDashboard.putData("Run Flywheel @ 15% speed, Kicker @ 50% speed",
          new RunShooter(m_shooter, 0.15, .50, true));
      SmartDashboard.putData("Jam", runKickerReverse());
    }

    if (m_intake != null) {
      SmartDashboard.putData("Extend Intake",
          new RunIntakeExtension(m_intake, 0.10, false));
      SmartDashboard.putData("Retract Intake",
          new RunIntakeExtension(m_intake, 0.10, true));
    }

    if (m_indexer != null) {
      SmartDashboard.putData("Reverse Indexer", new RunIndexer(m_indexer, 0.1, false));
    }

    if (m_hood != null) {
      SmartDashboard.putData("Move Hood to 15 degrees",
          new PivotHoodToPosition(m_hood, 0.10, 15, true));
    }
    // TODO: Index Jam Prevention Sequence Low Priority
  }

  private void addSysIdButtonsToSmartDashboard() {
    if (m_shooter != null) {
      SmartDashboard.putData("Flywheel QF",
          m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      SmartDashboard.putData("Flywheel QR",
          m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      SmartDashboard.putData("Flywheel DF",
          m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
      SmartDashboard.putData("Flywheel DR",
          m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    SmartDashboard.putData("(LIN) Drivebase QF", m_drivebase.sysIdQuasistatic(m_drivebase, IDrivebase.Mode.Linear,
        Direction.kForward));
    SmartDashboard.putData("(LIN) Drivebase QR", m_drivebase.sysIdQuasistatic(m_drivebase, IDrivebase.Mode.Linear,
        Direction.kReverse));
    SmartDashboard.putData("(LIN) Drivebase DF", m_drivebase.sysIdDynamic(m_drivebase, IDrivebase.Mode.Linear,
        Direction.kForward));
    SmartDashboard.putData("(LIN) Drivebase DR", m_drivebase.sysIdDynamic(m_drivebase, IDrivebase.Mode.Linear,
        Direction.kReverse));

    SmartDashboard.putData("(ANG) Drivebase QF",
        m_drivebase.sysIdQuasistatic(m_drivebase, IDrivebase.Mode.Angular, Direction.kForward));
    SmartDashboard.putData("(ANG) Drivebase QR",
        m_drivebase.sysIdQuasistatic(m_drivebase, IDrivebase.Mode.Angular, Direction.kReverse));
    SmartDashboard.putData("(ANG) Drivebase DF", m_drivebase.sysIdDynamic(m_drivebase, IDrivebase.Mode.Angular,
        Direction.kForward));
    SmartDashboard.putData("(ANG) Drivebase DR", m_drivebase.sysIdDynamic(m_drivebase, IDrivebase.Mode.Angular,
        Direction.kReverse));
  }

  public Command runKickerReverse() {
    return Commands.sequence(new RunShooterForTime(m_shooter, 0, 0.75, false, 2), new WaitCommand(0.5),
        new RunShooter(m_shooter, .15, .50, true));
  }

  // public Command runIndexerUnjam(){

  // return Commands.sequence(new)

  // }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
   * with an arbitrary predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link
   * edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers
   * or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  Trigger switchDriveTrigger;

  private void configureBindings() {
    m_arcadeDriveLeftStick = () -> {
      double scaling = getDriveSpeedScalingFactor();
      double axis = getDriverAxis(Constants.LogitechDualshock.LeftYAxis);
      if (m_switchDrive) {
        double joystickPercent = axis * scaling;
        return m_speedSlewRateLimiter.calculate(joystickPercent);
      } else {
        double joystickPercent = -axis * scaling;
        return m_speedSlewRateLimiter.calculate(joystickPercent);
      }
    };
    m_arcadeDriveRightStick = () -> {
      double scaling = getDriveSpeedScalingFactor();
      double axis = getDriverAxis(Constants.LogitechDualshock.RightXAxis);
      double joystickPercent = -axis * scaling;
      return m_rotSlewRateLimiter.calculate(joystickPercent);
    };

    switchDriveTrigger = new Trigger(() -> m_driverController.getRawButton(
        Constants.LogitechDualshock.BButton))
        .onTrue(
            new InstantCommand(() -> {
              m_switchDrive = !m_switchDrive;
            }));
    // Syntax for speed suppliers:
    // () -> m_driverController.getRawAxis(0)

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    ((Subsystem) m_drivebase).setDefaultCommand(new ArcadeDrive(
        m_arcadeDriveLeftStick, m_arcadeDriveRightStick, m_drivebase));
    LinearSpeedCommand setLinearSpeed = new LinearSpeedCommand(m_drivebase);
    SmartDashboard.putData("LinearSpeedCommand", setLinearSpeed);
  }

  private void configureDriverButtons() {
    if (m_intake != null) {
      new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.LeftTrigger))
          .whileTrue(new RunIntakeRollers(m_intake, 0.9, false));
      new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.RightTrigger))
          .whileTrue(new RunIntakeRollers(m_intake, 0.9, true));

      new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.StartButton))
          .whileTrue(new RunIntakeExtension(m_intake, 0.1, true));
      new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.BackButton))
          .whileTrue(new RunIntakeExtension(m_intake, 0.1, false));
    }
  }

  private double getDriveSpeedScalingFactor() {
    final boolean isTurtle = m_driverController.getRawButton(
        Constants.LogitechDualshock.LeftShoulder);
    final boolean isTurbo = m_driverController.getRawButton(
        Constants.LogitechDualshock.RightShoulder);

    if (isTurtle) {
      return Constants.RobotSpeedScaling.TURTLE_SPEED_SCALING;
    } else if (isTurbo) {
      return Constants.RobotSpeedScaling.TURBO_SPEED_SCALING;
    } else {
      return Constants.RobotSpeedScaling.NORMAL_SPEED_SCALING;
    }
  }

  private double getDriverAxis(int controller) {
    double axis = m_driverController.getRawAxis(controller);
    return (Math.abs(axis) < DEADBAND_CONSTANT) ? 0 : axis;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Implement functionality for autonomous mode.
    return Commands.print("We should do something in auto mode....");
  }
}
