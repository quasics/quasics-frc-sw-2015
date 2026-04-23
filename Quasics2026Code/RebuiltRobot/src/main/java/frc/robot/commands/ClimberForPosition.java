// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IClimber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberForPosition extends Command {

  IClimber m_climber;
  private final double m_climberSpeed;
  private double distanceExtend = 30; // Maximum height the robot can reach. Please change as needed.
  private double totalRunDistance;
  private final double firstRunDistance;
  private double climberHeight; // The current height it climber is at
  private final double m_go;
  final int beginDir; // Based on whether m_go is + or - ; either -1 ir 1.
  int loopDir; // Based on whether the height is decreasing or increasing (ex. if the climber
               // is going 200% of its heigt, it will go up, hit its max and then start
               // counting down.)

  public double TempTotalDistance(double firstValue, double ValueMore) { // find total distance for functions inside the
                                                                         // command
    double x = ValueMore - firstValue;
    return x;
  }

  /** Creates a new RunIntake. */
  public ClimberForPosition(IClimber climber, double climberSpeed, double goDistance) {

    m_go = goDistance;
    m_climber = climber;
    m_climberSpeed = climberSpeed;

    if (m_go < 0) {
      beginDir = -1;
    } else {
      beginDir = 1;
    }

    firstRunDistance = m_climber.getClimberPosition();
    ;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements((SubsystemBase) climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopDir = 0;

    double d = m_climber.getClimberPosition();
    ;
    if (d >= 0) {
      int looptimes = 0;
      while (d > 0) {

        d = d - (looptimes * m_climber.getClimberPosition());
        looptimes++;
      }
      d = climberHeight;
    }
    if (d < 0) {
      int looptimes = 0;
      while (d > 0) {

        d = d - (looptimes * m_climber.getClimberPosition());
        looptimes++;
      }
      d = climberHeight;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setClimberSpeed(m_climberSpeed);
    totalRunDistance = firstRunDistance - m_climber.getClimberPosition();
    ;

    if (-1 == beginDir * loopDir) {
      final double a = m_climber.getClimberPosition();
      ;
      do {
        climberHeight = TempTotalDistance(a, m_climber.getClimberPosition()) - distanceExtend;
      } while (climberHeight <= 0);
      loopDir = 1;
    }
    ;

    if (1 == beginDir * loopDir) {
      final double a = m_climber.getClimberPosition();
      ;
      do {
        climberHeight = TempTotalDistance(a, m_climber.getClimberPosition());
      } while (climberHeight >= 0);
      loopDir = 1;
    }

    System.out.println(climberHeight);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (totalRunDistance >= m_go) {
      m_climber.stopClimber();
    }

  }

  // TODO: Actually make distance work;
  // TODO: Make sure the static thingies aren't not working;
  // TODO: figure out what else you have to do

}
