package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.MoveClimbers;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTransitionRoller;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.TransitionRoller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Trajectorygenerator {
    public static Command GetcommandForTrajectory(String fileToLoad, Drivebase drivebase) {
        return new PrintCommand("TODO");
    }

    public static Command GetTrajectoryInitialPose(String fileToLoad) {
        return new PrintCommand("TODO");
    }

    public static Command GetTrajectoryFinalPose(String fileToLoad) {
        return new PrintCommand("TODO");
    }
}
