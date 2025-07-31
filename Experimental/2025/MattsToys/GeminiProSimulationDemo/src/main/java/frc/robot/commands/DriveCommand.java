package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends Command {
  private final Drivetrain m_drivetrain;
  private final XboxController m_driverController;

  /**
   * Creates a new DriveCommand.
   *
   * @param drivetrain The drivetrain subsystem this command will control.
   * @param driverController The XboxController used for driver input.
   */
  public DriveCommand(Drivetrain drivetrain, XboxController driverController) {
    m_drivetrain = drivetrain;
    m_driverController = driverController;

    // We must require the drivetrain subsystem, so no other command can use it
    // simultaneously.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is active.
  @Override
  public void execute() {
    // Get joystick values for speed (left stick Y) and rotation (right stick X)
    // Note: XboxController .getLeftY() returns negative for forward, so we
    // negate it.
    //       .getRightX() returns positive for right, which is standard for
    //       rotation.
    double xSpeed = -m_driverController.getLeftY();
    double zRotation = m_driverController.getRightX();

    // Deadband to ignore small joystick inputs near center
    final double DEADZONE = 0.1;
    if (Math.abs(xSpeed) < DEADZONE) {
      xSpeed = 0;
    }
    if (Math.abs(zRotation) < DEADZONE) {
      zRotation = 0;
    }

    m_drivetrain.arcadeDrive(xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop(); // Stop the motors when the command ends
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // This command runs indefinitely until interrupted (e.g., by
                  // another command)
  }
}
