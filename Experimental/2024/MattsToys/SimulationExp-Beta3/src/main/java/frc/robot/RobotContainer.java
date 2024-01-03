// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.RainbowLighting;
import frc.robot.commands.SpinInPlace;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.AbstractDrivebase;
import frc.robot.subsystems.RealDrivebase;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.LightingInterface;
import frc.robot.subsystems.RomiDrivebase;
import frc.robot.subsystems.SimulationDrivebase;
import frc.robot.subsystems.XrpDrivebase;
import frc.robot.utils.TrajectoryCommandGenerator;
import java.util.List;
import java.util.function.Supplier;

public class RobotContainer {
  final static int LIGHTING_PWM_PORT = 9;
  final static int NUM_LIGHTS = 40;

  static final double MAX_AUTO_VELOCITY_MPS = 3;
  static final double MAX_AUTO_ACCELERATION_MPSS = 1;
  static final TrajectoryConfig AUTO_SPEED_PROFILE = new TrajectoryConfig(MAX_AUTO_VELOCITY_MPS,
      MAX_AUTO_ACCELERATION_MPSS);

  private final XboxController m_controller = new XboxController(0);
  private final LightingInterface m_lighting = new Lighting(LIGHTING_PWM_PORT, NUM_LIGHTS);
  private final AbstractDrivebase m_drivebase;
  private final TrajectoryCommandGenerator m_trajectoryCommandGenerator;

  private enum SimulationMode {
    eSimulation, eXrp, eRomi
  }

  private final SimulationMode m_simulationMode = SimulationMode.eSimulation;

  final Supplier<Double> m_arcadeDriveForwardStick;
  final Supplier<Double> m_arcadeDriveRotationStick;

  public RobotContainer() {
    if (Robot.isReal()) {
      m_drivebase = new RealDrivebase();

      // Note that we're inverting the values because Xbox controllers return
      // negative values when we push forward.
      m_arcadeDriveForwardStick = () -> -m_controller.getLeftX(); // getRawAxis(1 /* X axis 0, y axis 1 */); //
                                                                  // getLeftX();
      m_arcadeDriveRotationStick = () -> -m_controller.getRightY(); // getRawAxis(5 /* X axis 4, y axis 5 */); //
                                                                    // getRightY();
    } else {
      // Note that we're assuming a keyboard-based controller is actually being used
      // in the simulation environment (for now), and thus we want to use axis 1&2
      // (without inversion, since it's *not* an Xbox controller).
      m_arcadeDriveForwardStick = () -> m_controller.getRawAxis(0);
      m_arcadeDriveRotationStick = () -> m_controller.getRawAxis(1);
      switch (m_simulationMode) {
        case eRomi:
          m_drivebase = new RomiDrivebase();
          break;
        case eXrp:
          m_drivebase = new XrpDrivebase();
          break;
        case eSimulation:
          m_drivebase = new SimulationDrivebase();
          break;
        default:
          System.err.println("**** WARNING: Unrecognized simulation mode (" + m_simulationMode
              + "), falling back on pure simulator");
          m_drivebase = new SimulationDrivebase();
          break;
      }
    }
    m_trajectoryCommandGenerator = new TrajectoryCommandGenerator(m_drivebase);
    configureBindings();
  }

  private void configureBindings() {
    m_drivebase.setDefaultCommand(new ArcadeDrive(m_drivebase, m_arcadeDriveForwardStick, m_arcadeDriveRotationStick));
    m_lighting.setDefaultCommand(new RainbowLighting(m_lighting));
  }

  private enum AutoModeTrajectorySelection {
    eControlSystemExampleFunctionalCommand,
    eControlSystemExampleRamseteCommand,
    eTrajectoryCommandGeneratorExample
  }

  private static final AutoModeTrajectorySelection m_autoModeTrajectorySelection = AutoModeTrajectorySelection.eControlSystemExampleRamseteCommand;

  /** Defines options for auto mode. */
  private enum AutoMode {
    eSpin, eFollowTrajectory
  }

  /** The currently-configured autonomous mode. */
  private AutoMode mode = AutoMode.eSpin;

  public Command getAutonomousCommand() {
    switch (mode) {
      case eSpin:
        return new SpinInPlace(m_drivebase);
      case eFollowTrajectory:
        return generateAutoModeSCurveCommand();
      default:
        return new PrintCommand("*****\n***** Error: Unhandled AutoMode setting!\n*****");
    }
  }

  private Command generateFunctionalCommandForControlSystemSampleTrajectory() {
    final Trajectory t = TrajectoryGenerator.generateTrajectory(new Pose2d(2, 2, new Rotation2d()),
        List.of(), new Pose2d(6, 4, new Rotation2d()), new TrajectoryConfig(2, 2));
    final RamseteController ramsete = new RamseteController();
    final Timer timer = new Timer();
    FunctionalCommand cmd = new FunctionalCommand(
        // init()
        () -> {
          m_drivebase.resetOdometry(t.getInitialPose());
          timer.restart();
        },
        // execute
        () -> {
          double elapsed = timer.get();
          Trajectory.State reference = t.sample(elapsed);
          ChassisSpeeds speeds = ramsete.calculate(m_drivebase.getPose(), reference);
          m_drivebase.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        },
        // end
        (Boolean interrupted) -> {
          m_drivebase.stop();
        },
        // isFinished
        () -> {
          return false;
        },
        // requiremnets
        m_drivebase);
    return cmd;
  }

  private Command generateRamseteCommandForControlSystemSampleTrajectory() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    final DifferentialDriveVoltageConstraint voltageConstraints = new DifferentialDriveVoltageConstraint(
        m_drivebase.getMotorFeedforward(), m_drivebase.getKinematics(), /* maxVoltage= */ 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(MAX_AUTO_VELOCITY_MPS, MAX_AUTO_ACCELERATION_MPSS)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_drivebase.getKinematics())
    // // Apply the voltage constraint
    // // NOTE: THE FAILURE SEEMS TO BE COMING FROM IN HERE!!!!
    // .addConstraint(voltageConstraints)
    // End of constraints
    ;

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_drivebase::getPose,
        new RamseteController(2, 0.7), m_drivebase.getMotorFeedforward(),
        m_drivebase.getKinematics(), m_drivebase::getWheelSpeeds,
        new PIDController(m_drivebase.getKP(), 0, 0), new PIDController(m_drivebase.getKP(), 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivebase::setMotorVoltages, m_drivebase);

    // Reset odometry to the starting pose of the trajectory.
    m_drivebase.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivebase.stop());
  }

  private Command generateAutoModeSCurveCommand() {
    switch (m_autoModeTrajectorySelection) {
      case eControlSystemExampleFunctionalCommand:
        return generateFunctionalCommandForControlSystemSampleTrajectory();
      case eControlSystemExampleRamseteCommand:
        return generateRamseteCommandForControlSystemSampleTrajectory();
      case eTrajectoryCommandGeneratorExample:
        return m_trajectoryCommandGenerator.generateCommand(AUTO_SPEED_PROFILE,
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)), false);
      default:
        return new PrintCommand("Unexpected value for m_autoModeTrajectorySelection!");
    }
  }
}
