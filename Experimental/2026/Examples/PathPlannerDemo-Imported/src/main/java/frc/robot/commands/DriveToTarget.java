// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.drivebase.IDrivebase;
import frc.robot.subsystems.interfaces.vision.IVisionPlus;

/**
 * Simple example of driving to a target.
 *
 * Note that this particular sample assumes that the target is starting out in
 * the robot's field of
 * view. This could be combined with code like that in <code>TurnToTarget</code>
 * in order to find
 * a target and then drive to it.
 *
 * @see https://docs.photonvision.org/en/latest/docs/examples/aimandrange.html
 */
public class DriveToTarget extends Command {
  /** Vision subsystem. */
  private final IVisionPlus m_vision;

  /** Drivebase subsystem. */
  private final IDrivebase m_drivebase;

  /** ID of the target to which the robot should drive. */
  private final int m_targetId;

  /** Determines if we'll print debugging output while command is active. */
  private final boolean m_noisy;

  /**
   * Determines if the command is finished. (Updated in initialize() and
   * execute().)
   */
  private boolean m_finished = false;

  /**
   * Angle in the view that we'd like to have on the target.
   *
   * Note that this is the angle to the center line of the robot to which we'd
   * like the target to be
   * aligned (e.g., "10 degrees means that it should that many degrees off from
   * our front axis").
   *
   * It does not guarantee that our robot's front face will be parallel (flat-on)
   * to the target.
   * This is basically because with a differential drive, we can't align our
   * robot's front face to
   * be parallel to the target, and then strafe from side-to-side in order to
   * ensure that we're
   * centered on it. (We'd need swerve drive for that.)
   */
  private static final Angle DESIRED_ANGLE = Degrees.of(0);

  /** Maximum acceptable error in angle of the target. */
  private static final Angle MAX_ANGLE_ERR = Degrees.of(3);

  /** Desired distance from the target. */
  private static final Distance DESIRED_DISTANCE = Meters.of(1.0);

  /** Maximum acceptable error in distance from the target. */
  private static final Distance MAX_DISTANCE_ERR = Meters.of(0.02);

  /**
   * kP to use in adjusting the robot's linear speed as we get closer. (Must be
   * tuned, based on
   * speed.)
   */
  private static final double FORWARD_KP = 0.25;

  /**
   * kP to use in adjusting the robot's turning speed as the target comes into
   * alignment. (Must be
   * tuned, based on speed.)
   */
  private static final double TURNING_KP = 0.008;

  /**
   * Constructor.
   *
   * @param vision    vision subsystem, supplying target data
   * @param drivebase drive base subsystem, allowing movement
   * @param targetId  target to which we should align/approach
   */
  public DriveToTarget(IVisionPlus vision, IDrivebase drivebase, int targetId) {
    this(vision, drivebase, targetId, false);
  }

  /**
   * Constructor.
   *
   * @param vision    vision subsystem, supplying target data
   * @param drivebase drive base subsystem, allowing movement
   * @param targetId  target to which we should align/approach
   * @param noisy     if true, generate debugging output
   */
  public DriveToTarget(IVisionPlus vision, IDrivebase drivebase, int targetId, boolean noisy) {
    m_vision = vision;
    m_drivebase = drivebase;
    m_targetId = targetId;
    m_noisy = noisy;

    addRequirements(m_vision.asSubsystem(), m_drivebase.asSubsystem());
  }

  @Override
  public void initialize() {
    if (!m_vision.isTargetVisible(m_targetId)) {
      System.err.println("**** ERROR: DriveToTarget(" + m_targetId
          + ") started, but can't see the target.  (Currently in view: "
          + m_vision.getVisibleTargets() + ")");
      m_finished = true;
      return;
    }

    m_finished = false;
  }

  @Override
  public void execute() {
    var targetData = m_vision.getTargetData(m_targetId);
    if (targetData == null) {
      System.err.println("**** Bailing out: Lost sight of target " + m_targetId
          + ".  (Currently in view: " + m_vision.getVisibleTargets() + ")");
      m_finished = true;
      return;
    }

    final var targetYaw = targetData.angle();
    final var yawError = DESIRED_ANGLE.minus(targetYaw);
    final var targetRange = targetData.distance();
    final var rangeError = targetRange.minus(DESIRED_DISTANCE);

    if (yawError.lte(MAX_ANGLE_ERR) && yawError.gte(MAX_ANGLE_ERR.times(-1))
        && rangeError.lte(MAX_DISTANCE_ERR) && rangeError.gte(MAX_DISTANCE_ERR.times(-1))) {
      // In range!
      if (m_noisy) {
        System.out.println("Close enough, at " + targetYaw + " and " + targetRange);
      }
      m_finished = true;
      return;
    }

    final double yawScaler = MathUtil.clamp(yawError.in(Degrees) * TURNING_KP, -1.0, +1.0);
    final var turnSpeed = IDrivebase.MAX_ROTATION.times(yawScaler);

    final double forwardScaler = MathUtil.clamp(rangeError.in(Meters) * FORWARD_KP, -1.0, +1.0);
    final var forwardSpeed = IDrivebase.MAX_SPEED.times(forwardScaler);
    if (m_noisy) {
      System.out.println("targetYaw: " + targetYaw + ", yawScaler: " + yawScaler
          + ", turnSpeed: " + turnSpeed.in(DegreesPerSecond) + ", targetRange: " + targetRange
          + ", forwardSpeed: " + forwardSpeed);
    }
    m_drivebase.arcadeDrive(forwardSpeed, turnSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivebase.stop();
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
