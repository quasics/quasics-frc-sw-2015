// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveteamConstants;
import frc.robot.Constants.PwmPortIds;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.PivotHoodToPosition;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntakeExtension;
import frc.robot.commands.RunIntakeRollers;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunShooterForTime;
import frc.robot.commands.RunShooterPID;
import frc.robot.commands.ShootBasedOnDistance;
import frc.robot.commands.lighting.RainbowLighting;
import frc.robot.commands.testing.DriveForDistance;
import frc.robot.commands.testing.FlywheelDialIn;
import frc.robot.commands.testing.LinearSpeedCommand;
import frc.robot.subsystems.interfaces.IClimber;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IIndexer;
import frc.robot.subsystems.interfaces.IIntake;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.IShooter;
import frc.robot.subsystems.interfaces.IShooterHood;
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
import java.util.List;
import java.util.function.Supplier;

import javax.swing.RowFilter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final int SIDE_LIGHTING_LENGTH = Constants.LIGHTING_TOTAL_LENGTH / 2;

  //
  // Support for a limited mechanism of establishing per-robot configuration
  // control.
  //

  /**
   * Names of Quasics robots on which this code might be executed.
   */
  enum RobotName {
    Simulated, Lizzie, Sally
  }

  /**
   * Identifies the default robot that we'll assume is in use when we're not
   * under simulation.
   */
  private static final RobotName DEFAULT_ROBOT_NAME = RobotName.Lizzie;

  /**
   * The robot name we'll *actually* use while executing (which will account for
   * simulation).
   */
  private static final RobotName ROBOT_NAME = Robot.isReal() ? DEFAULT_ROBOT_NAME : RobotName.Simulated;

  private static final boolean ENABLE_SHOOTER_TEST_CMDS = true;
  private static final boolean ENABLE_INDEXER_TEST_CMDS = true;
  private static final boolean ENABLE_HOOD_TEST_CMDS = true;
  private static final boolean ENABLE_INTAKE_TEST_CMDS = true;

  //
  // The robot's subystems are listed here.
  //

  private final IDrivebase m_drivebase = switch (ROBOT_NAME) {
    case Sally -> new SparkDriveBase();
    case Lizzie -> new NovaDriveBase();
    case Simulated -> new SimulationDrivebase();
  };
  private final IVision m_vision = switch (ROBOT_NAME) {
    case Lizzie -> new Vision();
    case Sally -> new IVision.NullVision();
    case Simulated -> new SimulatedVision();
  };
  private final IIntake m_intake = switch (ROBOT_NAME) {
    case Lizzie -> new RealIntake();
    case Sally, Simulated -> new IIntake.NullIntake();
  };
  private final IIndexer m_indexer = switch (ROBOT_NAME) {
    case Lizzie -> new RealIndexer();
    case Sally, Simulated -> new IIndexer.NullIndexer();
  };
  private final IShooterHood m_hood = switch (ROBOT_NAME) {
    case Lizzie -> new RealShooterHood();
    case Sally, Simulated -> new IShooterHood.NullShooterHood();
  };
  private final IShooter m_shooter = switch (ROBOT_NAME) {
    case Lizzie -> new RealShooter();
    case Sally, Simulated -> new IShooter.NullShooter();
  };
  private final IClimber m_climber = switch (ROBOT_NAME) {
    case Lizzie -> new RealClimber();
    case Sally, Simulated -> new IClimber.NullClimber();
  };

  /**
   * Primary lighting control (owns the LED strip and will partition it out).
   */
  private final ILighting m_primaryLighting;

  /** Subset of lighting targeted for the left side of the robot. */
  private final ILighting m_leftSideLighting;

  /** Subset of lighting targeted for the right side of the robot. */
  private final ILighting m_rightSideLighting;

  //
  // Operator control definitions.
  //

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController = new Joystick(DriveteamConstants.DRIVER_JOYSTICK_ID);
  private final Joystick m_operatorController = new Joystick(DriveteamConstants.OPERATOR_JOYSTICK_ID);

  /** Deadband range for reading data from driver/operator controllers. */
  private final double DEADBAND_CONSTANT = 0.08;

  /**
   * Iff true, switch drive is active (i.e., logical forward/backward for
   * driving the robot are swapped).
   */
  private boolean m_switchDrive = false;

  private final Autos m_autos = new Autos(m_drivebase);

  /**
   * The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    System.out.println("******************\n"
        + "Setting up robot for " + ROBOT_NAME + "\n"
        + "******************\n");

    // Don't warn about joysticks not being plugged in when working on a robot
    // where we frequently aren't worried about it.
    if (ROBOT_NAME == RobotName.Sally || ROBOT_NAME == RobotName.Sally) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    //
    // Finish allocating our subsystems and setting them up.
    //
    m_primaryLighting = allocatePrimaryLighting();
    m_leftSideLighting = allocateSideLighting(true);
    m_rightSideLighting = allocateSideLighting(false);

    m_leftSideLighting.setAsDefaultCommand(
        new RainbowLighting(m_leftSideLighting));
    m_rightSideLighting.setAsDefaultCommand(
        new RainbowLighting(m_rightSideLighting));

    // Connect cross-subsystem suppliers (so that the systems don't know about
    // each other directly)
    m_vision.setReferencePositionSupplier(() -> {
      if (m_drivebase != null) {
        return m_drivebase.getOdometryPose();
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

    // Populate the smart dashboard.
    addButtonsToSmartDashboard();

    // Configure the trigger bindings, etc.
    configureBindings();
    configureArcadeDriving();
    configureDriverButtons();
    configureOperatorButtons();

    SmartDashboard.putData("Simple start (blue)",
        Autos.generateSampleStartingCommand(m_drivebase, m_shooter,
            new Pose2d(Constants.RebuiltFieldData.BLUE_STARTING_LINE,
                Constants.RebuiltFieldData.MID_BUMP1_Y,
                new Rotation2d(Constants.RebuiltFieldData.FACING_BLUE))));

    SmartDashboard.putData("Simple start (red)",
        Autos.generateSampleStartingCommand(m_drivebase, m_shooter,
            new Pose2d(Constants.RebuiltFieldData.RED_STARTING_LINE,
                Constants.RebuiltFieldData.MID_BUMP2_Y,
                new Rotation2d(Constants.RebuiltFieldData.FACING_RED))));

    NamedCommands.registerCommand(
        "Shooter", new RunShooterForTime(m_shooter, 480, 120, true, 4));
    // NamedCommands.registerCommand("");
  }

  private ILighting allocatePrimaryLighting() {
    return new Lighting(PwmPortIds.LIGHTING_ID, Constants.LIGHTING_TOTAL_LENGTH,
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

    return new LightingBuffer(
        realLighting.getSubViews().get(targetIndex), isLeftSide);
  }

  private void addShooterTestCommandsToSmartDashboard() {
    if (m_shooter == null || !ENABLE_SHOOTER_TEST_CMDS) {
      return;
    }
    SmartDashboard.putData("Run Flywheel @ 1200 RPM, Kicker @ 12.5% speed",
        new RunShooterPID(m_shooter, RPM.of(1200), .125, 1));
    SmartDashboard.putData("Run Flywheel @ 3700 RPM, Kicker @ 38.7% speed",
        new RunShooterPID(m_shooter, RPM.of(3700), .387, 1));
    SmartDashboard.putData("Run Flywheel @ 15% speed, Kicker @ 50% speed",
        new RunShooter(m_shooter, 0.15, .50, true));
    SmartDashboard.putData("Jam", runKickerReverse());
    SmartDashboard.putData(
        "3500 RPM", new RunShooterPID(m_shooter, RPM.of(3500), .387, 1));
    SmartDashboard.putData(
        "3300 RPM", new RunShooterPID(m_shooter, RPM.of(3300), .387, 1));
    SmartDashboard.putData(
        "2700 RPM", new RunShooterPID(m_shooter, RPM.of(2700), .387, 1));
    SmartDashboard.putData(
        "3050 RPM", new RunShooterPID(m_shooter, RPM.of(3050), .387, 1));
    SmartDashboard.putData("Dial in Shooter", new FlywheelDialIn(m_shooter));
  }

  private void addIntakeTestCommandsToSmartDashboard() {
    if (m_intake == null || !ENABLE_INTAKE_TEST_CMDS) {
      return;
    }

    SmartDashboard.putData(
        "Run Intake Rollers", new RunIntakeRollers(m_intake, 0.1, true));
    SmartDashboard.putData("Run Indexer", new RunIndexer(m_indexer, 0.1, true));
    // SmartDashboard.putData("Extend Intake", new InstantCommand(() ->
    // m_intake.setExtensionSpeed(0.5)));
    // SmartDashboard.putData("Extend Intake", new InstantCommand(() ->
    // m_intake.setExtensionSpeed(-0.5)));
    SmartDashboard.putData(
        "Extend Intake", new RunIntakeExtension(m_intake, 0.10, false));
    SmartDashboard.putData(
        "Retract Intake", new RunIntakeExtension(m_intake, 0.20, true));
  }

  private void addIndexerTestCommandsToSmartDashboard() {
    if (m_indexer == null || !ENABLE_INDEXER_TEST_CMDS) {
      return;
    }
    SmartDashboard.putData(
        "Reverse Indexer", new RunIndexer(m_indexer, 0.1, false));
    // TODO: Index Jam Prevention Sequence Low Priority
  }

  private void addHoodTestCommandsToSmartDashboard() {
    if (m_hood == null || !ENABLE_HOOD_TEST_CMDS) {
      return;
    }
    SmartDashboard.putData("Move Hood to 15 degrees (60 degrees)",
        new PivotHoodToPosition(m_hood, 0.15, Degrees.of(15)));
    SmartDashboard.putData("Move Hood to 5 degrees (70 degrees)",
        new PivotHoodToPosition(m_hood, 0.15, Degrees.of(5)));
    SmartDashboard.putData("Move Hood to 25 degrees (50 degrees)",
        new PivotHoodToPosition(m_hood, 0.15, Degrees.of(25)));
  }

  private void addClimberTestCommandsToSmartDashboard() {
    System.out.println("CLIMBER: " + m_climber);
    if (m_climber == null) {
      return;
    }
    SmartDashboard.putData("Run climber @ 10%", 
      new RunClimber(m_climber, .1));
    SmartDashboard.putData("Run climber @ - 10%",
        new RunClimber(m_climber, -0.1));

//********************** */
    SmartDashboard.putData("Drive forward like a lot", 
      new DriveForDistance(m_drivebase, .2, meters.of(4)));
         //**************** */
    
    SmartDashboard.putData("Direction Climb Test",
        new DiretionalTestClimber());
  } 
    private Command DirectionalTestClimber() {
    return Commands.sequence(
        new RunClimberForTime(m_climber, 0.04, 0.5),
       new RunClimberForTime(m_climber, -0.04, 0.5));
      // (Playstation/Logitech) Y up, A down
  }

  private void addDrivebaseTestCommandsToSmartDashboard() {
    if (m_drivebase == null) {
      return;
    }
    SmartDashboard.putData("Braking on", new InstantCommand(() -> {
      m_drivebase.setBreakingMode(true);
    }, (Subsystem) m_drivebase));
    SmartDashboard.putData("Braking off", new InstantCommand(() -> {
      m_drivebase.setBreakingMode(false);
    }, (Subsystem) m_drivebase));
    SmartDashboard.putData(
        "LinearSpeedCommand", new LinearSpeedCommand(m_drivebase));
    SmartDashboard.putData("CMD: Testing encoders",
        Commands.sequence(
            new DriveForDistance(m_drivebase, .25, Meters.of(5))));
    SmartDashboard.putData("Align to Hub", new AlignToHub(m_drivebase));
  }

  private void addSysIdButtonsToSmartDashboard() {
    // Shooter characterization support
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

    // Drivebase characterization support
    if (m_drivebase != null) {
      SmartDashboard.putData("(LIN) Drivebase QF",
          m_drivebase.sysIdQuasistatic(
              m_drivebase, IDrivebase.Mode.Linear, Direction.kForward));
      SmartDashboard.putData("(LIN) Drivebase QR",
          m_drivebase.sysIdQuasistatic(
              m_drivebase, IDrivebase.Mode.Linear, Direction.kReverse));
      SmartDashboard.putData("(LIN) Drivebase DF",
          m_drivebase.sysIdDynamic(
              m_drivebase, IDrivebase.Mode.Linear, Direction.kForward));
      SmartDashboard.putData("(LIN) Drivebase DR",
          m_drivebase.sysIdDynamic(
              m_drivebase, IDrivebase.Mode.Linear, Direction.kReverse));

      SmartDashboard.putData("(ANG) Drivebase QF",
          m_drivebase.sysIdQuasistatic(
              m_drivebase, IDrivebase.Mode.Angular, Direction.kForward));
      SmartDashboard.putData("(ANG) Drivebase QR",
          m_drivebase.sysIdQuasistatic(
              m_drivebase, IDrivebase.Mode.Angular, Direction.kReverse));
      SmartDashboard.putData("(ANG) Drivebase DF",
          m_drivebase.sysIdDynamic(
              m_drivebase, IDrivebase.Mode.Angular, Direction.kForward));
      SmartDashboard.putData("(ANG) Drivebase DR",
          m_drivebase.sysIdDynamic(
              m_drivebase, IDrivebase.Mode.Angular, Direction.kReverse));
    }
  }

  private void addButtonsToSmartDashboard() {
    addShooterTestCommandsToSmartDashboard();
    addIntakeTestCommandsToSmartDashboard();
    addIndexerTestCommandsToSmartDashboard();
    addHoodTestCommandsToSmartDashboard();
    addClimberTestCommandsToSmartDashboard();
    addDrivebaseTestCommandsToSmartDashboard();
    addSysIdButtonsToSmartDashboard();
  }

  private Command runKickerReverse() {
    return Commands.sequence(
        new RunShooterForTime(m_shooter, 0, 0.75, false, 2),
        new WaitCommand(0.5), new RunShooter(m_shooter, .15, .50, true));
  }

  private void configureArcadeDriving() {
    // Slew rate limits for driving controls.
    final SlewRateLimiter speedSlewRateLimiter = new SlewRateLimiter(1);
    final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(1);

    // Set up speed suppliers for arcade driving
    //
    // Syntax for speed suppliers:
    // () -> m_driverController.getRawAxis(0)

    Supplier<Double> linearDrivingStick = () -> {
      double scaling = getDriveSpeedScalingFactor();
      double axis = getDriverAxis(Constants.LogitechDualshock.LeftYAxis);
      if (m_switchDrive) {
        double joystickPercent = axis * scaling;
        return speedSlewRateLimiter.calculate(joystickPercent);
      } else {
        double joystickPercent = -axis * scaling;
        return speedSlewRateLimiter.calculate(joystickPercent);
      }
    };
    final double ROTATION_FIXED_SCALING = 1.25;
    Supplier<Double> rotationDrivingStick = () -> {
      double scaling = getDriveSpeedScalingFactor();
      double axis = getDriverAxis(Constants.LogitechDualshock.RightXAxis);
      double joystickPercent = -axis * scaling * ROTATION_FIXED_SCALING;
      return rotationSlewRateLimiter.calculate(joystickPercent);
    };

    // Set up arcade driving as the default command for the drivebase.
    ((Subsystem) m_drivebase)
        .setDefaultCommand(new ArcadeDrive(
            linearDrivingStick, rotationDrivingStick, m_drivebase));
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
  private void configureBindings() {
    // Note that we're not saving the trigger in a variable. That's fine: this
    // is a "just set it up and let it do its thing" sort of thing.... new
    // Trigger(() ->
    // m_driverController.getRawButton(Constants.LogitechDualshock.BButton)).onTrue(new
    // InstantCommand(() -> {m_switchDrive = !m_switchDrive;}));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  private Command againstHubShot() {
    return Commands.sequence(
        new PivotHoodToPosition(m_hood, 0.15, Degrees.of(5)),
        new RunShooterPID(m_shooter, RPM.of(2700), .387, 1));
  }

  private Command towerShot() {
    return Commands.sequence(
        new PivotHoodToPosition(m_hood, 0.15, Degrees.of(15)),
        new RunShooterPID(m_shooter, RPM.of(3600), .387, 2));
  }

  private Command trenchShot() {
    return Commands.sequence(
        new PivotHoodToPosition(m_hood, 0.15, Degrees.of(15)),
        new RunShooterPID(m_shooter, RPM.of(3300), .387, 2));
  }

  private void configureDriverButtons() {
    if (m_intake != null) {
      new Trigger(() -> m_driverController.getRawButton(
          Constants.LogitechDualshock.LeftTrigger))
          .whileTrue(new RunIntakeRollers(m_intake, 0.55, false));
      new Trigger(() -> m_driverController.getRawButton(
          Constants.LogitechDualshock.RightTrigger))
          .whileTrue(new RunIntakeRollers(m_intake, 0.55, true));

      new Trigger(() -> m_driverController.getRawButton(
          Constants.LogitechDualshock.XButton))
          .whileTrue(new RunIntakeExtension(m_intake, 0.2, false));
      new Trigger(() -> m_driverController.getRawButton(
          Constants.LogitechDualshock.BButton))
          .whileTrue(new RunIntakeExtension(m_intake, 0.1, true));  
    }
    
    if(m_climber != null) {
      new Trigger(() -> m_driverController.getRawButton(    
          Constants.LogitechDualshock.YButton)) 
          .whileTrue(new RunClimber(m_climber, 0.1));
      new Trigger(() -> m_driverController.getRawButton(
          Constants.LogitechDualshock.AButton))
        .whileTrue(new RunIntakeExtension(m_climber, -0.1));    
    }
    new Trigger(() -> m_driverController.getRawButton(Constants.LogitechDualshock.StartButton))
        .whileTrue(new AlignToHub(m_drivebase));
  }

  private void configureOperatorButtons() {
    if (m_shooter != null) {
      new Trigger(() -> m_operatorController.getRawButton(
          XboxController.Button.kX.value))
          .whileTrue(towerShot());
      new Trigger(() -> m_operatorController.getRawButton(
          XboxController.Button.kB.value))
          .whileTrue(againstHubShot());
      // new Trigger(() ->
      // m_operatorController.getRawButton(XboxController.Button.kA.value)).whileTrue(trenchShot());
    }
    if (m_indexer != null) {
      new Trigger(() -> m_operatorController.getRawButton(
          XboxController.Button.kLeftBumper.value))
          .whileTrue(new RunIndexer(m_indexer, 0.5, true));
      new Trigger(() -> m_operatorController.getRawButton(
          XboxController.Button.kRightBumper.value))
          .whileTrue(new RunIndexer(m_indexer, 0.6, false));
    }

    if (m_shooter != null) {
      // FINDME(Rylie, Daniel): This isn't going to do what you think it will.
      // It's going to capture the distance to the hub's center when the driver
      // buttons are being set up (i.e., when the RobotContainer is being
      // built). However, it's unlikely that this will be the robot's distance
      // from the Hub when the driver actually *trigger* the command.
      //
      // Suggestion: instead of getting the distance *here*, write a Command
      // class that will get the distance while it is executing. This would at
      // least be when the initialize() function is invoked, but it would be
      // better to do it in execute() so that the target speed will be updated
      // continuously (e.g., if the robot is actually *moving* while we're
      // trying to shoot).
      new Trigger(() -> m_operatorController.getRawButton(
          XboxController.Button.kY.value))
          .whileTrue(
              new ShootBasedOnDistance(m_shooter, m_drivebase, 0.387, 2));
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
    return m_autos.getAuto();
  }
}
