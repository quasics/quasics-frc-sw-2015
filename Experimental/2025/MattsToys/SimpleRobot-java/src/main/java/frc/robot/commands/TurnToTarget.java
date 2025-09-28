// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IVisionPlus;

/**
 * Simple command to demonstrate using camera data to align with a target.
 *
 * Note that one nice improvement would be to scale our turning speed (or use PID control) to
 * provide more refined motion control as we try to align with the target, once it's in view.
 * (For example, scale using the measured angle to the target as our error.)
 */
public class TurnToTarget extends Command {
  /** Vision subsystem. */
  private final IVisionPlus m_vision;
  /** Drive subsystem. */
  private final IDrivebase m_drivebase;
  /** ID of the desired target. */
  private final int m_targetId;

  /** (Command state) Are we aligned (pointed directly at) the target? */
  private boolean m_aligned = false;

  /** Determines if we'll print debugging output while command is active. */
  static private final boolean NOISY = true;

  /** How closely aligned (+/-) we need to be with the target before we'll stop turning. */
  static private final Angle MIN_ACCEPTABLE_ANGLE = Degrees.of(3);

  // Note: positive values for turning are CCW in direction.
  /** How fast we should turn when we can't see the target at all. */
  static private final AngularVelocity SEEKING_SPEED = DegreesPerSecond.of(30);
  /** Max speed we should turn when the target is in view. */
  static private final AngularVelocity TRACKING_SPEED = DegreesPerSecond.of(5);

  /**
   * Creates a new TurnToTarget.
   *
   * @param vision vision subsystem (used to find the target)
   * @param drivebase drive subsystem (used to turn to target)
   * @param targetId ID for the desired target
   */
  public TurnToTarget(IVisionPlus vision, IDrivebase drivebase, int targetId) {
    this.m_vision = vision;
    this.m_drivebase = drivebase;
    this.m_targetId = targetId;

    addRequirements(vision.asSubsystem(), drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    // Assume that we can't see it at the moment.  (We'll figure out what to do in execute().)
    m_aligned = false;
  }

  @Override
  public void execute() {
    final var targetData = m_vision.getTargetData(m_targetId);
    if (targetData == null) {
      // Trivial case: it's not currently in view, so just turn until we (hopefully) find it
      m_drivebase.arcadeDrive(null, SEEKING_SPEED);
      return;
    }

    String updateMsg = ""; // Used to summarize action
    AngularVelocity turnSpeed = null; // Speed we need to turn at
    if (targetData.angle().gt(MIN_ACCEPTABLE_ANGLE)) {
      // Too far off (positive angle, so to the right of forward)
      turnSpeed = TRACKING_SPEED.times(-1);
      updateMsg = " - turning right (CW)";
    } else if (targetData.angle().lt(MIN_ACCEPTABLE_ANGLE.times(-1))) {
      // Too far off (negative angle, so to the left of forward)
      turnSpeed = TRACKING_SPEED.times(+1);
      updateMsg = " - turning left (CCW)";
    } else {
      // In the zone
      m_aligned = true;
      turnSpeed = IDrivebase.ZERO_TURNING;
      updateMsg = " - stopping!";
    }

    // Log results (if enabled)
    if (NOISY) {
      System.out.println("In view: " + targetData.angle().in(Degrees) + " (need +/-"
          + MIN_ACCEPTABLE_ANGLE.in(Degrees) + ")" + updateMsg);
    }

    // Update drive base
    m_drivebase.arcadeDrive(null, turnSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  @Override
  public boolean isFinished() {
    return m_aligned;
  }
}
