package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.CanBusIds.PIGEON2_CAN_ID;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class RealDrivebase extends AbstractDrivebase {
    // Common physical characteristics for Quasics' robots (and directly derived
    // values).
    static final Distance ANDYMARK_6IN_PLACTION_DIAMETER = Inches.of(6.0);
    static final Distance WHEEL_CIRCUMFERENCE = ANDYMARK_6IN_PLACTION_DIAMETER.times(Math.PI);
    static final double GEAR_RATIO = 8.45;

    ////////////////////////////////////////
    // Hardware control/sensing.
    //

    // Motors
    final SparkMax m_leftLeader = new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless);
    final SparkMax m_rightLeader = new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless);
    // final SparkMax m_leftFollower = new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID,
    // MotorType.kBrushless);
    // final SparkMax m_rightFollower = new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID,
    // MotorType.kBrushless);

    // Encoders
    private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

    // Gyro
    private final Pigeon2 m_rawGyro = new Pigeon2(PIGEON2_CAN_ID);

    public RealDrivebase() {
        super();

        ////////////////////////////////////////
        // Configure the encoders.
        System.out.println("Wheel circumference (m): " + WHEEL_CIRCUMFERENCE.in(Meters));

        // Conversion factor from units in rotations (or RPM) to meters (or m/s).
        final double distanceScalingFactorForGearing = WHEEL_CIRCUMFERENCE.div(GEAR_RATIO).in(Meters);
        final double velocityScalingFactor = distanceScalingFactorForGearing / 60;
        System.out.println("Using gear ratio: " + GEAR_RATIO);
        System.out.println("Adjustment for gearing (m/rotation): " + distanceScalingFactorForGearing);
        System.out.println("Velocity adj.: " + velocityScalingFactor);

        SparkMaxConfig leftConfig = getBaselineConfig();
        SparkMaxConfig rightConfig = getBaselineConfig();
        rightConfig.inverted(true);

        m_leftLeader.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private SparkMaxConfig getBaselineConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        final double distanceScalingFactor = WHEEL_CIRCUMFERENCE.div(GEAR_RATIO).in(Meters);
        final double velocityScalingFactor = distanceScalingFactor / 60;

        config.idleMode(IdleMode.kCoast);

        // Adjust the encoders to report in meters and meters/sec.
        config.encoder
                .positionConversionFactor(
                        distanceScalingFactor)
                .velocityConversionFactor(velocityScalingFactor);

        // TODO: Enable closed-loop PID control once we have the encoders working.
        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(1.0, 0.0, 0.0);

        return config;
    }

    @Override
    public void tankDrive(double leftPercentage, double rightPercentage) {
        m_leftLeader.set(leftPercentage);
        m_rightLeader.set(rightPercentage);
    }

    @Override
    public double getLeftDistanceMeters() {
        return m_leftEncoder.getPosition();
    }

    @Override
    public double getRightDistanceMeters() {
        return m_rightEncoder.getPosition();
    }

    public LinearVelocity getLeftVelocityMetersPerSecond() {
        return MetersPerSecond.of(m_leftEncoder.getVelocity());
    }

    public LinearVelocity getRightVelocityMetersPerSecond() {
        return MetersPerSecond.of(m_rightEncoder.getVelocity());
    }

    @Override
    public double getHeadingInDegrees() {
        return m_rawGyro.getAngle();
    }
}
