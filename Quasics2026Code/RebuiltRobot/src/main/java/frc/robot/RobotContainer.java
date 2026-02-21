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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveteamConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.LinearSpeedCommand;
import frc.robot.subsystems.interfaces.IIndexer;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.subsystems.real.AbstractDrivebase;
import frc.robot.subsystems.real.RealIndexer;
import frc.robot.subsystems.real.RealIntake;
import frc.robot.subsystems.real.RealShooter;
import frc.robot.subsystems.real.SparkDriveBase;
import frc.robot.subsystems.real.Vision;
import frc.robot.subsystems.simulated.SimulatedVision;
import frc.robot.subsystems.simulated.SimulationDrivebase;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final IIntake m_intakeSubsystem = new RealIntake();
  private final IIndexer m_indexerSubsystem = new RealIndexer();

  // The robot's subsystems and commands are defined here...
  private final AbstractDrivebase m_drivebase = Robot.isReal() ? new SparkDriveBase() : new SimulationDrivebase();
  private final IVision m_vision = (Robot.isReal()) ? new Vision() : new SimulatedVision();
  private final RealShooter m_shooter = new RealShooter();

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
  }

  private void addButtonsToSmartDashboard() {
    SmartDashboard.putData("Run Flwyheel @ 1200 RPM", new InstantCommand(() -> m_shooter.setFlywheelRPM(RPM.of(1200))));
    SmartDashboard.putData("Run Flywheel @ 3700 RPM", new InstantCommand(() -> m_shooter.setFlywheelRPM(RPM.of(3700))));
    SmartDashboard.putData("Run Kicker @ 12.5% speed", new InstantCommand(() -> m_shooter.setKickerSpeed(0.125)));
    SmartDashboard.putData("Run Kicker @ 38.7% speed", new InstantCommand(() -> m_shooter.setKickerSpeed(.387)));

  }

  private void addSysIdButtonsToSmartDashboard() {
    SmartDashboard.putData("Flywheel Quasistatic Forward",
        m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Flywheel Quasistatic Reverse",
        m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Flywheel Dynamic Forward",
        m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Flywheel Dynamic Reverse",
        m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

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
    m_drivebase.setDefaultCommand(new ArcadeDrive(
        m_arcadeDriveLeftStick, m_arcadeDriveRightStick, m_drivebase));
    LinearSpeedCommand setLinearSpeed = new LinearSpeedCommand(m_drivebase);
    SmartDashboard.putData("LinearSpeedCommand", setLinearSpeed);
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
