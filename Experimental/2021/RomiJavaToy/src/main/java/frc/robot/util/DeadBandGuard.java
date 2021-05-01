package frc.robot.util;

public class DeadBandGuard implements PowerFunction {
    final PowerFunction rawPowerFunction;
    final double lowLimit;
    final double highLimit;

    public DeadBandGuard(PowerFunction rawPowerFunction, double limit) {
        this(rawPowerFunction, limit, -limit);
    }

    public DeadBandGuard(PowerFunction rawPowerFunction, double lowLimit, double highLimit) {
        this.rawPowerFunction = rawPowerFunction;
        this.lowLimit = Math.min(lowLimit, highLimit);
        this.highLimit = Math.max(lowLimit, highLimit);
    }

    @Override
    public double get() {
        double value = rawPowerFunction.get();
        if (value >= lowLimit && value <= highLimit) {
            return 0;
        }
        return value;
    }

}
