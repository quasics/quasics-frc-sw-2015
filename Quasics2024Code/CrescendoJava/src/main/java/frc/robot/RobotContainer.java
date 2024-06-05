// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MoveClimbers;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTransitionRoller;
import frc.robot.commands.TimedRunShooter;
import frc.robot.commands.TimedRunIntake;
import frc.robot.commands.TimedRunTransitionRoller;


import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.TransitionRoller;
import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Trajectorygenerator.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private boolean m_switchDrive = false;

  private final Drivebase m_drivebase = new Drivebase();
  private final Climbers m_climbers = new Climbers();
  private final IntakeRoller m_intakeRoller = new IntakeRoller();
  private final TransitionRoller m_transitionRoller = new TransitionRoller();
  private final Shooter m_shooter = new Shooter();

  private final boolean ARCADE_DRIVE = true; // false for tank drive

  Supplier<Double> m_tankDriveLeftStick;
  Supplier<Double> m_tankDriveRightStick;
  Supplier<Double> m_arcadeDriveLeftStick;
  Supplier<Double> m_arcadeDriveRightStick;

  private final Joystick m_driverController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);
  private final Joystick m_operatorController = new Joystick(Constants.DriveTeam.OPERATOR_JOYSTICK_ID);

  Trigger switchDriveTrigger;

  SendableChooser<String> m_overallOptions = new SendableChooser<String>();
  SendableChooser<String> m_positionOptions = new SendableChooser<String>();
  SendableChooser<String> m_score2Options = new SendableChooser<String>();
  SendableChooser<String> m_score3Options = new SendableChooser<String>();


  private final double DEADBAND_CONSTANT = 0.04;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    ConfigureDriverButtons();
    ConfigureOperatorButtons();
    addButtonsToSmartDashboard();
    addOverallSelectorToSmartDashboard();
    addAutonomousStartingPositionsToSmartDashboard();
    addScore2OptionsToSmartDashboard();
    addScore3OptionsToSmartDashboard();
  }



  private void addOverallSelectorToSmartDashboard() {
    m_overallOptions.setDefaultOption(Constants.AutonomousSelectedOperation.doNothing, Constants.AutonomousSelectedOperation.doNothing);
    m_overallOptions.addOption(Constants.AutonomousSelectedOperation.GTFO, Constants.AutonomousSelectedOperation.GTFO);
    m_overallOptions.addOption(Constants.AutonomousSelectedOperation.score1, Constants.AutonomousSelectedOperation.score1);
    m_overallOptions.addOption(Constants.AutonomousSelectedOperation.score1GTFO, Constants.AutonomousSelectedOperation.score1GTFO);
    m_overallOptions.addOption(Constants.AutonomousSelectedOperation.score2, Constants.AutonomousSelectedOperation.score2);
    m_overallOptions.addOption(Constants.AutonomousSelectedOperation.score2GTFO, Constants.AutonomousSelectedOperation.score2GTFO);
    m_overallOptions.addOption(Constants.AutonomousSelectedOperation.score3, Constants.AutonomousSelectedOperation.score3);
    m_overallOptions.addOption(Constants.AutonomousSelectedOperation.score3GTFO, Constants.AutonomousSelectedOperation.score3GTFO);
    m_overallOptions.addOption(Constants.AutonomousSelectedOperation.score4, Constants.AutonomousSelectedOperation.score4);

    SmartDashboard.putData("Overall operation", m_overallOptions);
  }

  private void addAutonomousStartingPositionsToSmartDashboard() {
    m_positionOptions.setDefaultOption(Constants.AutonomousStartingPositions.inFrontOfAmp, Constants.AutonomousStartingPositions.inFrontOfAmp);
    m_positionOptions.addOption(Constants.AutonomousStartingPositions.leftOfSpeaker, Constants.AutonomousStartingPositions.leftOfSpeaker);
    m_positionOptions.addOption(Constants.AutonomousStartingPositions.inFrontOfSpeaker, Constants.AutonomousStartingPositions.inFrontOfSpeaker);
    m_positionOptions.addOption(Constants.AutonomousStartingPositions.rightOfSpeaker, Constants.AutonomousStartingPositions.rightOfSpeaker);
    m_positionOptions.addOption(Constants.AutonomousStartingPositions.farField, Constants.AutonomousStartingPositions.farField);

    SmartDashboard.putData("Starting position", m_positionOptions);
  }

  private void addScore2OptionsToSmartDashboard() {
    m_score2Options.setDefaultOption(Constants.AutonomousScore2Options.none, Constants.AutonomousScore2Options.none);
    m_score2Options.addOption(Constants.AutonomousScore2Options.amp, Constants.AutonomousScore2Options.amp);
    m_score2Options.addOption(Constants.AutonomousScore2Options.leftOfSpeaker, Constants.AutonomousScore2Options.leftOfSpeaker);
    m_score2Options.addOption(Constants.AutonomousScore2Options.inFrontOfSpeaker, Constants.AutonomousScore2Options.inFrontOfSpeaker);
    m_score2Options.addOption(Constants.AutonomousScore2Options.rightOfSpeakerAllianceNote, Constants.AutonomousScore2Options.rightOfSpeakerAllianceNote);
    m_score2Options.addOption(Constants.AutonomousScore2Options.rightOfSpeakerCenterNote, Constants.AutonomousScore2Options.rightOfSpeakerCenterNote);

    SmartDashboard.putData("Score 2 option", m_score2Options);
  }

  private void addScore3OptionsToSmartDashboard() {
    m_score3Options.setDefaultOption(Constants.AutonomousScore3Options.none, Constants.AutonomousScore3Options.none);
    m_score3Options.addOption(Constants.AutonomousScore3Options.amp, Constants.AutonomousScore3Options.amp);
    m_score3Options.addOption(Constants.AutonomousScore3Options.leftOfSpeaker, Constants.AutonomousScore3Options.leftOfSpeaker);
    m_score3Options.addOption(Constants.AutonomousScore3Options.inFrontOfSpeakerAmpNote, Constants.AutonomousScore3Options.inFrontOfSpeakerAmpNote);
    m_score3Options.addOption(Constants.AutonomousScore3Options.inFrontOfSpeakerStageNote, Constants.AutonomousScore3Options.inFrontOfSpeakerStageNote);
    m_score3Options.addOption(Constants.AutonomousScore3Options.inFrontOfSpeakerCenterNote, Constants.AutonomousScore3Options.inFrontOfSpeakerCenterNote);
    m_score3Options.addOption(Constants.AutonomousScore3Options.rightOfSpeaker, Constants.AutonomousScore3Options.rightOfSpeaker);

    SmartDashboard.putData("Score 3 option", m_score3Options);
  }

  private void addButtonsToSmartDashboard() {
    SmartDashboard.putData("set motor 6V", new InstantCommand(() -> m_drivebase.setVoltages(6, 6)));
    SmartDashboard.putData("Reset odometry", new InstantCommand(() -> m_drivebase.resetOdometry()));
    SmartDashboard.putData("Transition Roller Forward", new RunTransitionRoller(m_transitionRoller, .1, true));
    SmartDashboard.putData("Intake Roller Forward", new RunIntake(m_intakeRoller, .5, true));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
    */

  private double getDriverAxis(int controllerCode) {
    double axis = m_driverController.getRawAxis(controllerCode);
    // dead band enforcer
    return (Math.abs(axis) > DEADBAND_CONSTANT) ? axis : 0;
  }
  
  private final SlewRateLimiter m_leftSpeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rightSpeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_arcadeSpeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(1);

  private void configureBindings() {
    m_tankDriveLeftStick = () -> {
      double axis = -getDriverAxis(Constants.LogitechGamePad.LeftYAxis);
      if (m_switchDrive) {
        double joystickPercentage = -axis *getDriveSpeedScalingFactor();
        return m_leftSpeedLimiter.calculate(joystickPercentage);
      }
      else {
        double joystickPercentage = axis * getDriveSpeedScalingFactor();
        return m_leftSpeedLimiter.calculate(joystickPercentage);
      }
    };

    m_tankDriveRightStick = () -> {
      double axis = -getDriverAxis(Constants.LogitechGamePad.RightYAxis);
      if (m_switchDrive) {
        double joystickPercentage = -axis *getDriveSpeedScalingFactor();
        return m_rightSpeedLimiter.calculate(joystickPercentage);
      }
      else {
        double joystickPercentage = axis * getDriveSpeedScalingFactor();
        return m_rightSpeedLimiter.calculate(joystickPercentage);
      }
    };


    m_arcadeDriveLeftStick = () -> {
      double axis = -getDriverAxis(Constants.LogitechGamePad.LeftYAxis);
      if (m_switchDrive) {
        double joystickPercentage = -axis * getDriveSpeedScalingFactor();
        return m_arcadeSpeedLimiter.calculate(joystickPercentage);
      }
      else {
        double joystickPercentage = axis * getDriveSpeedScalingFactor();
        return m_arcadeSpeedLimiter.calculate(joystickPercentage);
      }
    };

    m_arcadeDriveRightStick = () -> {
      double axis = -getDriverAxis(Constants.LogitechGamePad.RightXAxis);
      double joystickPercentage = axis * getDriveSpeedScalingFactor();
      return m_rotationLimiter.calculate(joystickPercentage);
    };

    switchDriveTrigger = new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.BButton)).onTrue(
      new InstantCommand(() -> m_switchDrive = !m_switchDrive));
    
    if (ARCADE_DRIVE) {
      m_drivebase.setDefaultCommand(new ArcadeDrive(m_drivebase, m_arcadeDriveLeftStick, m_arcadeDriveRightStick));
    }
    else {
      m_drivebase.setDefaultCommand(new TankDrive(m_drivebase, m_tankDriveLeftStick, m_tankDriveRightStick));
    }
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

private  Command IntakeHelperCommand(boolean takingin){
  return Commands.parallel(new RunTransitionRoller(m_transitionRoller, .5, takingin), new RunIntake(m_intakeRoller, .5, takingin));
}

public static Command shootingSequence(TransitionRoller transitionRoller, Shooter shooter, double power){
  return Commands.parallel(transitionDelay(transitionRoller), new RunShooter(shooter, power, true));
}

public static Command transitionDelay(TransitionRoller transitionRoller){
  return Commands.sequence(new WaitCommand(0.75), new RunTransitionRoller(transitionRoller, .5, true));
}



private void ConfigureDriverButtons(){
  Trigger ExtendClimber = new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.YButton)).whileTrue(new MoveClimbers(m_climbers, true));
  Trigger RetractClimber = new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.AButton)).whileTrue(new MoveClimbers(m_climbers, false));
  Trigger IntakeNote = new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.LeftTrigger)).whileTrue(IntakeHelperCommand(true));
  Trigger DropNote = new Trigger(() -> m_driverController.getRawButton(Constants.LogitechGamePad.RightTrigger)).whileTrue(IntakeHelperCommand(false));
}

private void ConfigureOperatorButtons(){
  Trigger SpeakerScoringSequence = new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kX.value)).whileTrue(shootingSequence(m_transitionRoller, m_shooter, 0.75));
  Trigger AmpScoringSequence = new Trigger(() -> m_operatorController.getRawButton(XboxController.Button.kB.value)).whileTrue(shootingSequence(m_transitionRoller, m_shooter, 0.25));
}




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String overallOperation = m_overallOptions.getSelected();
    String positionOption = m_positionOptions.getSelected();
    String score2Option = m_score2Options.getSelected();
    String score3Option = m_score3Options.getSelected();

    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    final boolean isBlue = alliance == DriverStation.Alliance.Blue;

    return Autos.getAutonomousCommand(m_drivebase, m_intakeRoller, m_transitionRoller, m_shooter, overallOperation, positionOption, score2Option, score3Option, isBlue);
  }
}
