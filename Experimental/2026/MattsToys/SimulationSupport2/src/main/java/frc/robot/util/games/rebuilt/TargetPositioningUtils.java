// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.games.rebuilt;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.games.RebuiltConstants;
import frc.robot.util.LinearInterpolator;

/**
 * Some potentially helpful functions for use in calculating targeting data for
 * the Rebuilt game.
 */
public class TargetPositioningUtils {
  /**
   * Returns the center position of the hub for the team's alliance.
   * 
   * @return the location (in field-based coordinates) of the hub
   */
  public static Translation2d getHubCenterLocation() {
    Optional<Alliance> optAlliance = DriverStation.getAlliance();
    if (optAlliance.isEmpty()) {
      return new Translation2d();
    }

    return getHubCenterLocation(optAlliance.get());
  }

  /**
   * Returns the center position of the specified alliance's hub.
   * 
   * @param alliance the alliance whose hub we care about
   * @return the location (in field-based coordinates) of the hub
   */
  public static Translation2d getHubCenterLocation(Alliance alliance) {
    Distance x = switch (alliance) {
      case Red -> RebuiltConstants.FIELD_LENGTH.minus(RebuiltConstants.HUB_CENTER_DISTANCE_TO_ALLIANCE_WALL);
      case Blue -> RebuiltConstants.HUB_CENTER_DISTANCE_TO_ALLIANCE_WALL;
    };

    Distance y = RebuiltConstants.FIELD_WIDTH.div(2);

    return new Translation2d(x, y);
  }

  /**
   * Returns the calculated distance to the center of the team's hub, given the
   * robot's current location.
   * 
   * @param robotPose robot's current pose
   * @return distance to the center of the team's hub
   */
  public static Distance getDistanceToHubCenter(Translation2d robotPosition) {
    Translation2d hubCenter = getHubCenterLocation();
    return Meters.of(robotPosition.getDistance(hubCenter));
  }

  /**
   * Returns the calculated distance to the center of the team's hub, given the
   * robot's current location.
   * 
   * @param robotPose robot's current pose
   * @return distance to the center of the team's hub
   */
  public static Distance getDistanceToHubCenter(Pose2d robotPose) {
    return getDistanceToHubCenter(robotPose.getTranslation());
  }

  /**
   * Computes an approximate speed for the shooter to use in hitting the hub's
   * center, based on the robot's current position (pose) and a previously
   * identified set of values (speedInterpolator).
   * 
   * @param robotPose         robot position
   * @param speedInterpolator interpolator object, loaded with
   *                          "distanceInMeters:speed" values
   * @return approximate speed needed to hit the alliance's hub center from the
   *         specified position
   */
  public static double getShooterSpeedForHubCenter(Pose2d robotPose, LinearInterpolator speedInterpolator) {
    final double distance = getDistanceToHubCenter(robotPose).in(Meters);
    return speedInterpolator.getTargetApproximationForKey(distance);
  }

  /**
   * Returns the field-relative heading needed in order for the robot to be
   * pointed at its alliance's hub (given the robot's current position).
   * 
   * @param robotPose current field-relative pose of the robot (e.g., based on
   *                  pose estimation)
   * @return the field-relative heading to the hub center (i.e., what you want to
   *         have as your heading in the robot's *pose*, if you're aiming at the
   *         hub's center)
   */
  public static Rotation2d getAngleToHubCenter(Pose2d robotPose) {
    return getAngleToHubCenter(robotPose.getTranslation());
  }

  /**
   * Returns the field-relative heading needed in order for the robot to be
   * pointed at its alliance's hub (given the robot's current position).
   * 
   * @param robotPosition current field-relative position of the robot (e.g.,
   *                      "getEstimatedPose().getTranslation()")
   * @return the field-relative heading to the hub center (i.e., what you want to
   *         have as your heading in the robot's *pose*, if you're aiming at the
   *         hub's center)
   */
  public static Rotation2d getAngleToHubCenter(Translation2d robotPosition) {
    final var hubCenterPosition = getHubCenterLocation();
    Translation2d relativeVector = hubCenterPosition.minus(robotPosition);
    return relativeVector.getAngle();
  }
}
