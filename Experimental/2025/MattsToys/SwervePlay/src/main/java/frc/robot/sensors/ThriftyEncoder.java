package frc.robot.sensors;

// FINAL CORRECTED Imports
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAnalogSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ThriftyEncoder {
    // REV's analog sensor class, configured for absolute mode
    private final SparkAnalogSensor analogSensor;

    // The unique angular offset (in rotations) to define the module's 0 degree
    // position
    private final double encoderOffsetRotations;

    /**
     * Initializes the ThriftyEncoder, assuming it's wired into the Spark Max's
     * analog port.
     * 
     * @param motor  The CANSparkMax the sensor is wired into.
     * @param offset The module's unique offset in rotations (0 to 1).
     */
    public ThriftyEncoder(SparkMax motor, double offset) {
        // Instantiate the Spark Max Analog Sensor in absolute mode
        analogSensor = motor.getAnalog();

        // Store the pre-determined offset
        encoderOffsetRotations = offset;
    }

    /**
     * Returns the continuous, field-relative angle of the module.
     * 
     * @return Rotation2d object representing the current angle, wrapped to [-180,
     *         180] degrees.
     */
    public Rotation2d getRotation() {
        // We rely on getPosition() to return the value scaled to 0-1 rotations.
        double rawRotations = analogSensor.getPosition();
        double normalizedRotations = rawRotations - encoderOffsetRotations;

        normalizedRotations = MathUtil.inputModulus(
                normalizedRotations, -0.5, 0.5);

        return Rotation2d.fromRotations(normalizedRotations);
    }
}
