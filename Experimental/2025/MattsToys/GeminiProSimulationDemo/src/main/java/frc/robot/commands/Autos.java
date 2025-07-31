package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands; // Utility class for simple commands
import frc.robot.subsystems.Drivetrain; // Import the Drivetrain subsystem

public final class Autos {
  /**
   * Example static factory for an autonomous command.
   * You can create many different autonomous routines here.
   *
   * @param drivetrain The Drivetrain subsystem this autonomous command will
   *     use.
   * @return An autonomous command.
   */
  public static Command exampleAuto(Drivetrain drivetrain) {
    // This is a simple example that does nothing.
    // In a real robot, you would chain commands here to perform your autonomous
    // routine. For instance, you could add: .andThen(new
    // DriveDistanceCommand(drivetrain, 1.0)) // Drive 1 meter .andThen(new
    // TurnDegreesCommand(drivetrain, 90));   // Turn 90 degrees
    return Commands.print("No autonomous command configured yet.")
        .andThen(Commands.runOnce(
            drivetrain::stop, drivetrain)); // Ensure drivetrain stops at end
  }

  // You can add more autonomous commands as static methods here:
  // public static Command driveStraightAuto(Drivetrain drivetrain) {
  //   return new DriveDistanceCommand(drivetrain, 2.0); // Example: Drive 2
  //   meters
  // }

  private Autos() {
    throw new UnsupportedOperationException(
        "This is a utility class and cannot be instantiated!");
  }
}
