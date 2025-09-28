// Copyright (c) FIRST and other WPILib contributors.
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

/* You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TurnToTarget extends Command {
  private final IVisionPlus m_vision;
  private final IDrivebase m_drivebase;
  private final int m_targetId;
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

  /**
   * Updates driving and "m_aligned", based on camera data.
   *
   * @see #initialize()
   * @see #execute()
   */
  private void update() {
    final var targetData = m_vision.getTargetData(m_targetId);
    if (targetData == null) {
      // Trivial case: it's not currently in view, so just turn until we (hopefully) find it
      m_drivebase.arcadeDrive(null, SEEKING_SPEED);
      return;
    }

    // Note: One nice improvement here would be to scale our turning speed (or use PID control) to
    // provide more refined motion as we try to align with the target (e.g., based on
    // targetData.angle().minus(MIN_ACCEPTABLE_ANGLE)).

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
  public void initialize() {
    m_aligned = false;
  }

  @Override
  public void execute() {
    update();
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
