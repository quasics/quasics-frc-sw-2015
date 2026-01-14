package frc.robot.misc;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.interfaces.ILighting;
import frc.robot.subsystems.interfaces.ILighting.StockColor;
import frc.robot.util.PoseHelpers;

public class FieldPlacementColorFunction implements ILighting.ColorSupplier {
    final Supplier<Pose2d> m_targetPoseSupplier;
    final Supplier<Pose2d> m_currentPoseSupplier;

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

    final static double ROTATION_ACCEPTABLE_ERROR_DEGREES = 1;
    final static double DISTANCE_ACCEPTABLE_ERROR_METERS = Inches.of(4).in(Meters);
    final static int SIGNAL_BAND_WIDTH = 4;

    @Override
    public Color getColorForLed(int position) {
        final Pose2d currentPose = m_currentPoseSupplier.get();
        final Pose2d targetPose = m_targetPoseSupplier.get();
        if (currentPose == null || targetPose == null) {
            return ILighting.StockColor.Green.toWpiColor();
        }

        StockColor result = StockColor.Green;
        var delta = PoseHelpers.computePoseDelta(targetPose, currentPose);
        if (position < SIGNAL_BAND_WIDTH) {
            // "Rotation" error
            if (delta.rotationDelta().getDegrees() > ROTATION_ACCEPTABLE_ERROR_DEGREES) {
                result = StockColor.Red;
            } else if (delta.rotationDelta().getDegrees() < -ROTATION_ACCEPTABLE_ERROR_DEGREES) {
                result = StockColor.Blue;
            }
        } else if (position < (SIGNAL_BAND_WIDTH * 2)) {
            // "X" error
            if (delta.xDelta().in(Meters) > DISTANCE_ACCEPTABLE_ERROR_METERS) {
                result = StockColor.Red;
            } else if (delta.xDelta().in(Meters) < -DISTANCE_ACCEPTABLE_ERROR_METERS) {
                result = StockColor.Blue;
            }
        } else if (position < (SIGNAL_BAND_WIDTH * 3)) {
            // "Y" error
            if (delta.yDelta().in(Meters) > DISTANCE_ACCEPTABLE_ERROR_METERS) {
                result = StockColor.Red;
            } else if (delta.yDelta().in(Meters) < -DISTANCE_ACCEPTABLE_ERROR_METERS) {
                result = StockColor.Blue;
            }
        }

        // Default color
        return result.toWpiColor();
    }

}
