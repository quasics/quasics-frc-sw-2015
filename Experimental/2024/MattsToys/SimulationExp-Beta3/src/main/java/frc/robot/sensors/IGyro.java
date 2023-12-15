package frc.robot.sensors;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogGyro;

/**
 * As of the 2024 tools, a common "Gyro" interface is being deprecated, and the
 * various classes for gyros/IMUs are now all left without a common base type
 * (which sucks, if you want to write code that can swap different hardware
 * in/out).
 * 
 * So, I'm putting in my own wrapper interface, along with some functionality to
 * help encapsulate various "real" gyro classes with the wrapper.
 */
public interface IGyro {
    void calibrate();

    /** Returns the heading of the robot in degrees. */
    double getAngle();

    /** Returns the rate of rotation of the gyro. */
    double getRate();

    /** Returns the heading of the robot as a Rotation2d. */
    Rotation2d getRotation2d();

    void reset();

    public class FunctionalGyro implements IGyro {
        private final Runnable m_calibrator;
        private final Supplier<Double> m_angleSupplier;
        private final Supplier<Double> m_rateSupplier;
        private final Supplier<Rotation2d> m_rotationSupplier;
        private final Runnable m_resetter;

        FunctionalGyro(Runnable calibrator, Supplier<Double> angleSupplier, Supplier<Double> rateSupplier,
                Supplier<Rotation2d> rotationSupplier, Runnable resetter) {
            m_calibrator = calibrator;
            m_angleSupplier = angleSupplier;
            m_rateSupplier = rateSupplier;
            m_rotationSupplier = rotationSupplier;
            m_resetter = resetter;
        }

        @Override
        public void calibrate() {
            m_calibrator.run();
        }

        @Override
        public double getAngle() {
            return m_angleSupplier.get();
        }

        @Override
        public double getRate() {
            return m_rateSupplier.get();
        }

        @Override
        public Rotation2d getRotation2d() {
            return m_rotationSupplier.get();
        }

        @Override
        public void reset() {
            m_resetter.run();
        }
    }

    static IGyro wrapYawGyro(Pigeon2 pigeon2) {
        return new FunctionalGyro(
                () -> {
                    System.out.println(">>> Null-op: Pigeon2 auto-calibrates.");
                },
                () -> pigeon2.getAngle(),
                () -> pigeon2.getRate(),
                () -> pigeon2.getRotation2d(),
                () -> {
                    // Note that this will do a reset on the Pigeon for *all* axes. A better
                    // approach might be to use something like the "OffsetGyro" approach that
                    // I prototyped in last year's "JavaUtilityLib", so that we can reset
                    // just this *view* of the device. (Though if someone resets the master
                    // device, we'd still be stuck with a similar problem.)
                    // TODO(mjh): Port something like the OffsetGyro into this year's
                    // examples.
                    pigeon2.reset();
                });
    }

    static IGyro wrapAnalogGyro(AnalogGyro g) {
        return new FunctionalGyro(
                () -> {
                    g.calibrate();
                },
                () -> g.getAngle(),
                () -> g.getRate(),
                () -> g.getRotation2d(),
                () -> {
                    g.reset();
                });
    }
}
