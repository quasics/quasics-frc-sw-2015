// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class TankDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_leftSpeedSupplier;
  private final Supplier<Double> m_rightSpeedSupplier;

  /**
   * Creates a new TankDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param leftSpeedSupplier Lambda supplier of left motor speed
   * @param rightSpeedSupplier Lambda supplier of right motor speed
   */
  public TankDrive(
      Drivetrain drivetrain,
      Supplier<Double> leftSpeedSupplier,
      Supplier<Double> rightSpeedSupplier) {
    m_drivetrain = drivetrain;
    m_leftSpeedSupplier = leftSpeedSupplier;
    m_rightSpeedSupplier = rightSpeedSupplier;
    addRequirements(drivetrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.tankDrive(m_leftSpeedSupplier.get(), m_rightSpeedSupplier.get());
  }
}
