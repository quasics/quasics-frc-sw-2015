// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IVisionPlus;

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TurnToTarget extends Command {
  private final IVisionPlus vision;
  private final IDrivebase drivebase;
  private final int targetId;

  static private final AngularVelocity SEEKING_SPEED = DegreesPerSecond.of(15);

  /** Creates a new TurnToTarget. */
  public TurnToTarget(IVisionPlus vision, IDrivebase drivebase, int targetId) {
    this.vision = vision;
    this.drivebase = drivebase;
    this.targetId = targetId;

    addRequirements(vision.asSubsystem(), drivebase.asSubsystem());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!vision.isTargetVisible(targetId)) {
      drivebase.arcadeDrive(null, SEEKING_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: Update this so that it's not just a question of visibility, but actual alignment.
    return vision.isTargetVisible(targetId);
  }
}
