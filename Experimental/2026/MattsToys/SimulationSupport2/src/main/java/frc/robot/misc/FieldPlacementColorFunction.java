// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.misc;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.ILighting.StockColor;
import frc.robot.util.PoseHelpers;

/**
 * Sample class to use in setting the light values on the robot while it is
 * disabled, indicating if the robot is positioned (x, y, heading) correctly for
 * use in auto mode.
 * 
 * In this example, blocks of N lights will be used to indicate if a particular
 * value needs to be adjusted:
 * <ul>
 * <li>First block is for rotation
 * <li>Second block is "x" offset (long dimension of the field, with 0 at the
 * blue end)
 * <li>Third block is "y" offset (short dimension of the field, measuring "out"
 * when the Blue alliance is on your left)
 * </ul>
 * 
 * Red values indicate that the error is in the negative direction (e.g., inward
 * from X or Y); blue values indicate that the error is in the positive
 * direction (too far out from X or Y).
 */
public class FieldPlacementColorFunction implements ILighting.ColorSupplier {
    /** Acceptable heading error (in degrees). */
    final static double ROTATION_ACCEPTABLE_ERROR_DEGREES = 1;

    /** Acceptable distance error (in meters). */
    final static double DISTANCE_ACCEPTABLE_ERROR_METERS = Inches.of(4).in(Meters);

    /** How many lights are used for each pose component (rotation, x, y). */
    final static int SIGNAL_BAND_WIDTH = 4;

    /** Default color for the lights in the strip. */
    final static StockColor DEFAULT_COLOR = StockColor.Green;

    /** Color used to indicate errors in the positive direction. */
    final static StockColor POSITIVE_ERROR_COLOR = StockColor.Blue;

    /** Color used to indicate errors in the negative direction. */
    final static StockColor NEGATIVE_ERROR_COLOR = StockColor.Red;

    /** Used to get the current target pose. */
    final Supplier<Pose2d> m_targetPoseSupplier;

    /** Used to get the robot's current pose on the field. */
    final Supplier<Pose2d> m_currentPoseSupplier;

    /**
     * Constructor.
     * 
     * @param targetPoseSupplier  supplies the target pose (e.g., based on the
     *                            starting position for a trajectory to be followed
     *                            in auto mode)
     * @param currentPoseSupplier supplies the robot's current pose on the field
     */
    public FieldPlacementColorFunction(Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> currentPoseSupplier) {
        if (targetPoseSupplier == null) {
            throw new IllegalArgumentException("targetPoseSupplier can't be null");
        }
        if (currentPoseSupplier == null) {
            throw new IllegalArgumentException("currentPoseSupplier can't be null");
        }

        m_targetPoseSupplier = targetPoseSupplier;
        m_currentPoseSupplier = currentPoseSupplier;
    }

    @Override
    public Color getColorForLed(int position) {
        final Pose2d currentPose = m_currentPoseSupplier.get();
        final Pose2d targetPose = m_targetPoseSupplier.get();
        if (currentPose == null || targetPose == null) {
            return DEFAULT_COLOR.toWpiColor();
        }

        StockColor result = DEFAULT_COLOR;
        var delta = PoseHelpers.computePoseDelta(targetPose, currentPose);
        if (position < SIGNAL_BAND_WIDTH) {
            // "Rotation" error
            if (delta.rotationDelta().getDegrees() > ROTATION_ACCEPTABLE_ERROR_DEGREES) {
                result = POSITIVE_ERROR_COLOR;
            } else if (delta.rotationDelta().getDegrees() < -ROTATION_ACCEPTABLE_ERROR_DEGREES) {
                result = NEGATIVE_ERROR_COLOR;
            }
        } else if (position < (SIGNAL_BAND_WIDTH * 2)) {
            // "X" error
            if (delta.xDelta().in(Meters) > DISTANCE_ACCEPTABLE_ERROR_METERS) {
                result = POSITIVE_ERROR_COLOR;
            } else if (delta.xDelta().in(Meters) < -DISTANCE_ACCEPTABLE_ERROR_METERS) {
                result = NEGATIVE_ERROR_COLOR;
            }
        } else if (position < (SIGNAL_BAND_WIDTH * 3)) {
            // "Y" error
            if (delta.yDelta().in(Meters) > DISTANCE_ACCEPTABLE_ERROR_METERS) {
                result = POSITIVE_ERROR_COLOR;
            } else if (delta.yDelta().in(Meters) < -DISTANCE_ACCEPTABLE_ERROR_METERS) {
                result = NEGATIVE_ERROR_COLOR;
            }
        }

        return result.toWpiColor();
    }
}
