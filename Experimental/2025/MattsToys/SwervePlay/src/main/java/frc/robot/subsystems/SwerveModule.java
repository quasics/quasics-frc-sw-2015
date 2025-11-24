package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.ModuleConstants;
import frc.robot.sensors.ThriftyEncoder;

public class SwerveModule {
  private final SparkMax driveMotor;
  private final SparkMax steerMotor;

  private final RelativeEncoder driveEncoder;
  private final ThriftyEncoder absoluteEncoder;

  private final SparkClosedLoopController steerPIDController;

  /**
   * Initializes the SwerveModule hardware abstraction layer.
   *
   * @param driveCANId CAN ID for the drive NEO.
   * @param steerCANId CAN ID for the steer NEO.
   * @param offset     The absolute encoder offset to define 0 degrees (in
   *                   rotations).
   */
  public SwerveModule(int driveCANId, int steerCANId, double offset) {
    // Instantiate Motors
    driveMotor = new SparkMax(driveCANId, MotorType.kBrushless);
    steerMotor = new SparkMax(steerCANId, MotorType.kBrushless);

    // 1. CREATE AND APPLY CONFIGURATION OBJECTS

    // Define Drive Motor Config
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(ModuleConstants.kDriveInverted);
    // Include other necessary motor settings (e.g., current limits, idle mode)
    // driveConfig.setSmartCurrentLimit(ModuleConstants.kDriveCurrentLimit);

    // Define Steer Motor Config
    SparkMaxConfig steerConfig = new SparkMaxConfig();
    steerConfig.inverted(ModuleConstants.kSteerInverted);
    // steerConfig.setSmartCurrentLimit(ModuleConstants.kSteerCurrentLimit);

    // Instantiate Encoders
    driveEncoder = driveMotor.getEncoder();
    absoluteEncoder = new ThriftyEncoder(steerMotor, offset);

    // Configure Steer Motor PID
    steerPIDController = steerMotor.getClosedLoopController();

    // 2. CREATE AND APPLY PID CONFIGURATION
    // Final Correction: PID parameters are set directly on the SparkMaxConfig
    steerConfig.closedLoop.p(ModuleConstants.kSteerP);
    steerConfig.closedLoop.i(ModuleConstants.kSteerI);
    steerConfig.closedLoop.d(ModuleConstants.kSteerD);
    steerConfig.closedLoop.outputRange(
        ModuleConstants.kMinSteerOutput, ModuleConstants.kMaxSteerOutput);

    // --- CRITICAL CONFIGURATION FOR THRIFTY ENCODER ---
    // Set the conversion factor for the auxiliary feedback sensor (Thrifty Encoder)
    // This scales the analog reading such that it is reported as 0-1 rotation.
    steerConfig.absoluteEncoder.positionConversionFactor(1.0);
    // ----------------------------------------------------

    // Final Integration Step: Synchronize NEO Position
    // Set the steer NEO's internal encoder position to the current absolute
    // reading.
    double absoluteAngleRotations = absoluteEncoder.getRotation().getRotations();
    double initialMotorPosition = absoluteAngleRotations * ModuleConstants.kSteerGearRatio;
    steerMotor.getEncoder().setPosition(initialMotorPosition);

    // Apply Configurations
    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    steerMotor.configure(
        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public SwerveModulePosition getPosition() {
    final double distanceMeters = driveEncoder.getPosition();
    final Rotation2d angle = absoluteEncoder.getRotation();
    return new SwerveModulePosition(distanceMeters, angle);
  }

  /**
   * Returns the current state (velocity and angle) of the module.
   *
   * @return The SwerveModuleState object.
   */
  public SwerveModuleState getState() {
    // Drive velocity is read from the integrated encoder (in meters/sec)
    // The VelocityConversionFactor must be set on the drive NEO for this to work
    // correctly.
    double driveVelocityMetersPerSec = driveEncoder.getVelocity() * ModuleConstants.kDriveVelocityConversionFactor;

    // Steer angle is read from the configured absolute encoder
    Rotation2d steerAngle = absoluteEncoder.getRotation();

    return new SwerveModuleState(driveVelocityMetersPerSec, steerAngle);
  }

  /**
   * Sets the desired state (speed and angle) for the module.
   *
   * @param desiredState The target SwerveModuleState (velocity and angle).
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // 1. Apply Optimization (Minimizes rotation angle)
    SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

    // 2. Set Drive Velocity (using Velocity PID/FF)
    // Convert the target linear velocity (m/s) to RPM for the drive PID.
    double targetRPM = optimizedState.speedMetersPerSecond / ModuleConstants.kDriveVelocityConversionFactor;

    // Assuming Velocity Control is used for the drive motor (setReference)
    driveMotor.getClosedLoopController().setReference(targetRPM, SparkBase.ControlType.kVelocity);

    // 3. Set Steer Position (using Position PID)
    // Convert the Rotation2d target angle to the target NEO encoder position
    // (rotations)
    double targetSteerPosition = optimizedState.angle.getRotations() * ModuleConstants.kSteerGearRatio;

    steerPIDController.setReference(targetSteerPosition, SparkBase.ControlType.kPosition);
  }
}
