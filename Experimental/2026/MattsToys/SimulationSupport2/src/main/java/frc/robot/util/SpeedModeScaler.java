package frc.robot.util;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

public class SpeedModeScaler implements UnaryOperator<Double> {
    final Supplier<SpeedMode> m_modeSupplier;
    final double m_normalScaling;
    final double m_turboScaling;
    final double m_turtleScaling;

    public SpeedModeScaler(Supplier<SpeedMode> modeSupplier, double normalScaling, double turboScaling,
            double turtleScaling) {
        if (modeSupplier == null) {
            throw new IllegalArgumentException("modeSupplier cannot be null");
        }
        if (turtleScaling > normalScaling) {
            throw new IllegalArgumentException("turtleScaling must be <= normalScaling");
        }
        if (normalScaling > turboScaling) {
            throw new IllegalArgumentException("normalScaling must be <= turboScaling");
        }
        m_modeSupplier = modeSupplier;
        m_normalScaling = normalScaling;
        m_turboScaling = turboScaling;
        m_turtleScaling = turtleScaling;
    }

    @Override
    public Double apply(Double t) {
        return switch (m_modeSupplier.get()) {
            case Normal -> t * m_normalScaling;
            case Turbo -> t * m_turboScaling;
            case Turtle -> t * m_turtleScaling;
        };
    }
}
