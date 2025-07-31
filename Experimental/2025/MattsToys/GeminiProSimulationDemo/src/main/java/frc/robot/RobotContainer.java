package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand; // Import your new command
import frc.robot.subsystems.Drivetrain; // Import your new subsystem

// !! REMOVE THIS IMPORT, NO LONGER NEEDED IF DRIVERSTATION ISN'T PASSED !!
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.DriverStation; // No longer needed here if not
// passing

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot "logic" or
 * "if/else" statements should actually be placed in this class. Instead, the
 * structure of the robot (including subsystems, commands, and button mappings)
 * should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // !! REVERT TO ORIGINAL DRIVETRAIN INSTANTIATION !!
  private final Drivetrain m_drivetrain =
      new Drivetrain(); // Instantiated without parameter

  // The driver's controller
  XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // !! REMOVE THESE LINES, NO LONGER NEEDED FOR DRIVERSTATION PASSING !!
    // DriverStation driverStation = RobotBase.getDriverStation();
    // m_drivetrain = new Drivetrain(driverStation); // Pass DriverStation to
    // Drivetrain

    // Configure the button bindings
    configureButtonBindings();

    // Set the default command for the drivetrain. This command will run
    // whenever no other command is currently requiring the drivetrain.
    m_drivetrain.setDefaultCommand(
        new DriveCommand(m_drivetrain, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one
   * of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link
   * XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Example: Driver can press A button to reset encoders (useful for testing)
    // new Trigger(m_driverController::getAButton)
    //     .onTrue(m_drivetrain::resetEncoders);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example autonomous command
    return Autos.exampleAuto(m_drivetrain); // If you modify Autos.java, you
                                            // might return a different command
  }
}
