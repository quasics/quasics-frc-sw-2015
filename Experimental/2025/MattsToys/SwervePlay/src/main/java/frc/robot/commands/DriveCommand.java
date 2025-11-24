package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController; // or Joystick
import edu.wpi.first.wpilibj2.command.RunCommand; // Preferred for this continuous command
import frc.robot.DriveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveCommand extends RunCommand {
  private final SwerveDriveSubsystem driveSubsystem;

  // Constants for deadband and scaling
  private static final double kDeadband = 0.05;
  private static final double kMaxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
  private static final double kMaxAngularSpeed = DriveConstants.kMaxAngularSpeedRadiansPerSecond;

  /**
   * Creates a new DriveCommand.
   * 
   * @param driveSubsystem  The swerve drive subsystem this command controls.
   * @param driveController The controller providing the driver input.
   */
  public DriveCommand(SwerveDriveSubsystem driveSubsystem, XboxController driveController) {
    // Use RunCommand constructor for continuous execution
    super(
        // The command's execution logic (What to do)
        () -> driveSubsystem.drive(
            // xSpeed (Forward/Backward) - Typically Left Stick Y
            getDeadbandInput(-driveController.getLeftY(), kDeadband) * kMaxSpeed,
            // ySpeed (Strafe/Sideways) - Typically Left Stick X
            getDeadbandInput(-driveController.getLeftX(), kDeadband) * kMaxSpeed,
            // rot (Rotation/Turning) - Typically Right Stick X
            getDeadbandInput(-driveController.getRightX(), kDeadband) * kMaxAngularSpeed,
            // Field Relative is set to true by default for standard swerve operation
            true),
        // The subsystem requirement (What is needed)
        driveSubsystem);

    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    // Ensure the SwerveDriveSubsystem has this as its default command
    driveSubsystem.setDefaultCommand(this);
  }

  // Simple static method to handle deadband and cubic scaling (for smoother
  // control)
  private static double getDeadbandInput(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0.0;
    }
    // Applying cubic scaling for more granular control near zero
    return Math.copySign(input * input * input, input);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }
}
