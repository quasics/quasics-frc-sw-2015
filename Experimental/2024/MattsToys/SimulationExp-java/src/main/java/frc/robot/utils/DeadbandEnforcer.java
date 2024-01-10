package frc.robot.utils;

public class DeadbandEnforcer {
    final private double m_minVal, m_maxVal;
    final private double m_deadVal;

    public DeadbandEnforcer(double minVal, double maxVal, double deadVal) {
        m_minVal = Math.min(minVal, maxVal);
        m_maxVal = Math.max(minVal, maxVal);
        m_deadVal = deadVal;
    }

    public DeadbandEnforcer(double minVal, double maxVal) {
        this(minVal, maxVal, 0);
    }

    public DeadbandEnforcer(double val) {
        this(-val, val);
    }

    public double limit(double val) {
        if (val <= m_minVal || val >= m_maxVal) {
            return val;
        }
        return m_deadVal;
    }
}