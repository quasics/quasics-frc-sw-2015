// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.LogitechGamePad;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmWaveCommand;
import frc.robot.commands.DriveForDistance;
import frc.robot.commands.MoveArmToAngle;
import frc.robot.commands.MoveElevatorToExtreme;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.subsystems.AbstractElevator;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.interfaces.IVision;
import frc.robot.subsystems.simulations.SimDrivebase;
import frc.robot.subsystems.simulations.SimulatedElevator;
import frc.robot.subsystems.simulations.SimulatedSingleJointArm;
import frc.robot.subsystems.simulations.SimulatedVision;
import frc.robot.utils.DeadbandEnforcer;
import frc.robot.utils.RobotConfigs;
import frc.robot.utils.RobotConfigs.RobotConfig;

import java.util.function.Supplier;

public class RobotContainer {
  final RobotConfigs.Robot DEPLOYED_ON = RobotConfigs.Robot.Simulation;
  final RobotConfig m_robotConfig = RobotConfigs.getConfig(DEPLOYED_ON);

  // Subsystems
  final IVision m_vision = new SimulatedVision(m_robotConfig);
  private final IDrivebase m_drivebase = new SimDrivebase();
  final AbstractElevator m_elevator = new SimulatedElevator();
  final ISingleJointArm m_arm = new SimulatedSingleJointArm();

  // Controllers
  //
  // TODO: Consider using CommandJoystick class instead of Joystick. (Would let me
  // explicitly bind specific channels for X/Y, possibly simplifying live vs
  // simulation handling, as well as directly providing "trigger factories" for
  // commands.)
  //
  // Note that live joysticks generally follow a different orientation/coordinate
  // system than the one used for the robot.
  //
  // @see
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
  private final Joystick m_driveController = new Joystick(Constants.DriveTeam.DRIVER_JOYSTICK_ID);

  public RobotContainer() {
    configureArcadeDrive();
    configureBindings();

    SmartDashboard.putData(
        "Wave arm",
        new ArmWaveCommand(m_arm));
    SmartDashboard.putData(
        "Arm out",
        new MoveArmToAngle(m_arm, ISingleJointArm.ARM_OUT_ANGLE_RADIANS));
    SmartDashboard.putData(
        "Arm up",
        new MoveArmToAngle(m_arm, ISingleJointArm.ARM_UP_ANGLE_RADIANS));
    SmartDashboard.putData(
        "Raise elevator (wait)",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.Top, true));
    SmartDashboard.putData(
        "Raise elevator (nowait)",
        new MoveElevatorToPosition(m_elevator, AbstractElevator.TargetPosition.Top, false));
  }

  private void configureArcadeDrive() {
    final DeadbandEnforcer deadbandEnforcer = new DeadbandEnforcer(Constants.DriveTeam.DRIVER_DEADBAND);
    Supplier<Double> forwardSupplier;
    Supplier<Double> rotationSupplier;

    if (Robot.isReal()) {
      // Configure the real robot.
      //
      // Note that we're inverting the values because Xbox controllers return
      // negative values when we push forward.
      forwardSupplier = () -> -deadbandEnforcer.limit(m_driveController.getRawAxis(LogitechGamePad.LeftYAxis));
      rotationSupplier = () -> -deadbandEnforcer.limit(m_driveController.getRawAxis(LogitechGamePad.RightXAxis));
    } else {
      // Configure the simulated robot
      //
      // Note that we're assuming a keyboard-based controller is actually being
      // used in the simulation environment (for now), and thus we want to use
      // axis 0&1 (from the "Keyboard 0" configuration).
      forwardSupplier = () -> deadbandEnforcer.limit(m_driveController.getRawAxis(0));
      rotationSupplier = () -> -deadbandEnforcer.limit(m_driveController.getRawAxis(1));
    }

    m_drivebase.asSubsystem().setDefaultCommand(
        new ArcadeDrive(m_drivebase, forwardSupplier, rotationSupplier));
  }

  private void configureBindings() {
    // TODO: Configure any bindings as required.
  }

  public Command getAutonomousCommand() {
    // Simple demo command to drive forward while raising the elevator.
    return new ParallelCommandGroup(
        new DriveForDistance(m_drivebase, .50, Meters.of(3)), new MoveElevatorToExtreme(m_elevator, true));
    // return Commands.print("No autonomous command configured");
  }
}
