// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;



import frc.robot.Constants.AutonomousSelectedOperation;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;

import frc.robot.subsystems.TransitionRoller;


import static edu.wpi.first.units.Units.Seconds;




public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto() {
    // TODO
    return new WaitCommand(0);
  }

  public static Command GTFO(Drivebase drivebase) {
    return new TimedMovementTest(drivebase, Seconds.of(2), 0.50);
  }

  public static Command score1(TransitionRoller transitionRoller, Shooter shooter) {
    return shootingSequence(transitionRoller, shooter);
  }

  public static Command intakeHelperCommand(IntakeRoller intakeRoller, TransitionRoller transitionRoller) {
    return Commands.parallel(new RunTransitionRoller(transitionRoller, 0.5, true), new RunIntake(intakeRoller, 0.5, true));
  }

  public static Command shootingSequence(TransitionRoller transitionRoller, Shooter shooter){
    return Commands.parallel(transitionDelay(transitionRoller), new TimedRunShooter(shooter, 0.5, Seconds.of(2.0), true));
  }

  public static Command transitionDelay(TransitionRoller transitionRoller){
    return Commands.sequence(new WaitCommand(0.75), new TimedRunTransitionRoller(transitionRoller, .5, Seconds.of(1.25), true));
  }
  
  public static Command getAutonomousCommand(Drivebase drivebase, IntakeRoller intakeRoller, TransitionRoller transitionRoller, Shooter shooter, String overallOperation, String positionOption, String score2Option, String score3Option) {
    //ArrayList<Command> commands = new ArrayList<Command>();
    if (overallOperation == AutonomousSelectedOperation.doNothing){
      return new PrintCommand("Doing Nothing");
    } else if (overallOperation == AutonomousSelectedOperation.GTFO){
      return GTFO(drivebase);
    } else if (overallOperation == AutonomousSelectedOperation.score1) {
      return score1(transitionRoller, shooter);
    }
    //return Commands.sequence(commands);
    return new PrintCommand("???");
  }
}
