package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
// Corrected Unit Imports for WPILIB 2025
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Encoder;
// Simulation specific imports
import edu.wpi.first.wpilibj.RobotBase;
// NEW IMPORT FOR ROBOTCONTROLLER
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.motorcontrol.SpeedControllerGroup; // !! REMOVE
// THIS IMPORT !!
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Motor Controllers
  // Designate front motors as masters, rear as followers
  private final PWMSparkMax m_leftFrontMotor =
      new PWMSparkMax(0); // PWM Port 0 (Master)
  private final PWMSparkMax m_leftRearMotor =
      new PWMSparkMax(1); // PWM Port 1 (Follower)
  private final PWMSparkMax m_rightFrontMotor =
      new PWMSparkMax(2); // PWM Port 2 (Master)
  private final PWMSparkMax m_rightRearMotor =
      new PWMSparkMax(3); // PWM Port 3 (Follower)

  // DifferentialDrive directly takes the master motors
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

  // Slew Rate Limiters for smoother driving (prevents sudden changes in motor
  // output)
  private final SlewRateLimiter m_rightSlewRateLimiter =
      new SlewRateLimiter(3.0); // 3.0 units/sec
  private final SlewRateLimiter m_leftSlewRateLimiter =
      new SlewRateLimiter(3.0); // 3.0 units/sec

  // Encoders (for simulation and potentially real robot odometry)
  // DIO ports for encoders - adjust based on your robot's wiring
  private final Encoder m_leftEncoder = new Encoder(4, 5);  // DIO Ports 4 and 5
  private final Encoder m_rightEncoder = new Encoder(6, 7); // DIO Ports 6 and 7

  // --- Simulation Specifics ---
  private DifferentialDrivetrainSim m_drivetrainSim;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private Field2d m_fieldSim = new Field2d();

  public Drivetrain() {
    // !! Configure followers !!
    m_leftRearMotor.addFollower(m_leftFrontMotor);
    m_rightRearMotor.addFollower(m_rightFrontMotor);

    // Invert one side of the drivetrain so that forward is positive for both
    // You apply inversion to the master motor, and followers will respect it.
    m_rightFrontMotor.setInverted(true);

    // Set distance per pulse for encoders (e.g., if 1 rotation is 12.56 inches
    // and encoder has 360 pulses) This value will depend on your robot's wheel
    // diameter and encoder resolution Using 2025 Units for this:
    Distance wheelCircumference = Inches.of(6.0).times(Math.PI);
    final double kEncoderDistancePerPulse =
        wheelCircumference.in(Units.Meters) /
        360.0; // 6 inch wheel, 360 CPR encoder
    m_leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);

    // --- Simulation Setup ---
    if (RobotBase.isSimulation()) {
      // Corrected unit conversions for WPILib 2025+ Units
      double massInKg = Pounds.of(140.0).in(Units.Kilograms);
      double trackwidthInMeters = Inches.of(25.0).in(Units.Meters);
      double wheelRadiusInMeters = Inches.of(3.0).in(Units.Meters);

      // CALCULATE MOMENT OF INERTIA (J)
      double robotLengthMeters = Inches.of(30.0).in(Units.Meters);
      double robotWidthMeters = Inches.of(28.0).in(Units.Meters);
      double momentOfInertiaKgMetersSquared =
          (1.0 / 12.0) * massInKg *
          (Math.pow(robotLengthMeters, 2) + Math.pow(robotWidthMeters, 2));

      m_drivetrainSim = new DifferentialDrivetrainSim(
          DCMotor.getNEO(2), // 2 NEO motors per side (this assumes 2 masters +
                             // 2 followers, total 4 motors. If you only have 2
                             // physical motors, adjust to 1 here)
          10.71,             // Gearing (e.g., 10.71:1 for Toughbox Mini)
          massInKg,          // Robot mass (converted from pounds to kilograms)
          momentOfInertiaKgMetersSquared, // Pass the calculated moment of
                                          // inertia
          trackwidthInMeters,  // Trackwidth (converted from inches to meters)
          wheelRadiusInMeters, // Wheel radius (converted from inches to meters)
          null); // This last null is for the "plant" argument, which is more
                 // advanced.

      // Initialize encoder simulations
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);

      // Put the Field2d object on SmartDashboard to visualize robot pose
      SmartDashboard.putData("Field", m_fieldSim);
    }
  }

  /**
   * Drives the robot using arcade drive.
   *
   * @param xSpeed The forward/backward speed (-1.0 to 1.0)
   * @param zRotation The rotation speed (-1.0 to 1.0)
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    // Apply slew rate limiters for smoother control
    double limitedXSpeed = m_leftSlewRateLimiter.calculate(
        xSpeed); // Use Left Slew for X (forward/backward)
    double limitedZRotation = m_rightSlewRateLimiter.calculate(
        zRotation); // Use Right Slew for Z (rotation)

    m_drive.arcadeDrive(limitedXSpeed, limitedZRotation);
  }

  /** Stops the drivetrain motors. */
  public void stop() {
    m_drive.stopMotor(); // This will stop the master motors, and followers will
                         // stop automatically
  }

  /** Resets the encoders to zero. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Returns the current distance of the left encoder in meters.
   *
   * @return The distance in meters.
   */
  public double getLeftEncoderDistance() { return m_leftEncoder.getDistance(); }

  /**
   * Returns the current distance of the right encoder in meters.
   *
   * @return The distance in meters.
   */
  public double getRightEncoderDistance() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run (typically every 20ms)
    // Here we'll update our simulation and SmartDashboard data.

    if (RobotBase.isSimulation()) {
      // Update the DrivetrainSim with the current motor inputs
      // Use the master motors' output for the simulation
      m_drivetrainSim.setInputs(
          m_leftFrontMotor.get() * RobotController.getBatteryVoltage(),
          m_rightFrontMotor.get() * RobotController.getBatteryVoltage());

      // Advance the simulation by 20ms (default robot loop time)
      m_drivetrainSim.update(0.02);

      // Update simulated encoders based on drivetrain simulation
      m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
      m_leftEncoderSim.setRate(
          m_drivetrainSim.getLeftVelocityMetersPerSecond());
      m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
      m_rightEncoderSim.setRate(
          m_drivetrainSim.getRightVelocityMetersPerSecond());

      // Update the Field2d widget with the simulated robot's pose
      m_fieldSim.setRobotPose(m_drivetrainSim.getPose());
    }

    // Put drivetrain status on SmartDashboard
    // Log the master motor outputs
    SmartDashboard.putNumber("Left Master Motor Output",
                             m_leftFrontMotor.get());
    SmartDashboard.putNumber("Right Master Motor Output",
                             m_rightFrontMotor.get());
    SmartDashboard.putNumber("Left Encoder Distance (m)",
                             getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Distance (m)",
                             getRightEncoderDistance());
    SmartDashboard.putNumber("Left Encoder Rate (m/s)",
                             m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Encoder Rate (m/s)",
                             m_rightEncoder.getRate());

    // You can add more telemetry here as your robot gets more complex
  }
}
