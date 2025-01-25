package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimulatedDrivebase extends AbstractDrivebase {
  public final int LEFT_DRIVE_PWM_ID = 0;
  public final int RIGHT_DRIVE_PWM_ID = 1;

  public final int GYRO_CHANNEL = 0;

  public final int LEFT_DRIVE_ENCODER_PORT_A = 0;
  public final int LEFT_DRIVE_ENCODER_PORT_B = 1;
  public final int RIGHT_DRIVE_ENCODER_PORT_A = 2;
  public static final int RIGHT_DRIVE_ENCODER_PORT_B = 3;

  public static final Distance kWheelRadius = Inches.of(6.0).div(2); // 6" diameter
  public static final Distance kRobotTrackWidth = Meters.of(0.381 * 2);
  public static final int kEncoderResolutionTicksPerRevolution = -4096;

  // "Hardware" allocation
  private final PWMSparkMax m_left = new PWMSparkMax(LEFT_DRIVE_PWM_ID);
  private final PWMSparkMax m_right = new PWMSparkMax(RIGHT_DRIVE_PWM_ID);
  private final Encoder m_leftEncoder =
      new Encoder(LEFT_DRIVE_ENCODER_PORT_A, LEFT_DRIVE_ENCODER_PORT_B);
  private final Encoder m_rightEncoder =
      new Encoder(RIGHT_DRIVE_ENCODER_PORT_A, RIGHT_DRIVE_ENCODER_PORT_B);
  private final AnalogGyro m_gyro = new AnalogGyro(GYRO_CHANNEL);

  /////////////////////////////////////////////////////////////////////////////////////
  // Simulated "hardware" and other simulation-specific objects.
  private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
  private final DifferentialDrivetrainSim m_drivetrainSimulator =
      new DifferentialDrivetrainSim(m_drivetrainSystem, DCMotor.getCIM(2), 8,
          kRobotTrackWidth.in(Meters), kWheelRadius.in(Meters), null);
  private final Field2d m_fieldSim = new Field2d();

  public SimulatedDrivebase() {
    super();

    //////////////////////////////////////////
    // Finish hardware setup
    //

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.setInverted(true);

    // Set the distance per pulse (in meters) for the drive encoders. We can simply
    // use the distance traveled for one rotation of the wheel divided by the
    // encoder resolution.
    m_leftEncoder.setDistancePerPulse(
        2 * Math.PI * kWheelRadius.in(Meters) / kEncoderResolutionTicksPerRevolution);
    m_rightEncoder.setDistancePerPulse(
        2 * Math.PI * kWheelRadius.in(Meters) / kEncoderResolutionTicksPerRevolution);

    // Make sure our encoders are zeroed out on startup.
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    //////////////////////////////////////////
    // Finish simulation setup
    //

    // Add the simulated field to the smart dashboard
    SmartDashboard.putData("Field", m_fieldSim);
  }

  @Override
  public void tankDrive(double leftPercentage, double rightPercentage) {
    // Clamp speeds to the range [-1.0, 1.0].
    leftPercentage = Math.max(-1.0, Math.min(1.0, leftPercentage));
    rightPercentage = Math.max(-1.0, Math.min(1.0, rightPercentage));

    m_left.set(leftPercentage);
    m_right.set(rightPercentage);
  }

  @Override
  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  @Override
  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public double getLeftVelocityMetersPerSecond() {
    return m_leftEncoder.getRate();
  }

  @Override
  public double getRightVelocityMetersPerSecond() {
    return m_rightEncoder.getRate();
  }

  @Override
  public double getHeadingInDegrees() {
    return m_gyro.getAngle();
  }

  @Override
  public void periodic() {
    super.periodic();

    // Update the position for the robot that is shown in the simulated field, using
    // the odometry data that will have been computed by the base class.
    m_fieldSim.setRobotPose(getPose2d());
  }

  // Note: this method will be called once per scheduler run
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(m_left.get() * RobotController.getInputVoltage(),
        m_right.get() * RobotController.getInputVoltage());

    // Simulated clock ticks forward
    m_drivetrainSimulator.update(0.02);

    // Update the encoders and gyro, based on what the drive train simulation says
    // happend.
    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }
}
