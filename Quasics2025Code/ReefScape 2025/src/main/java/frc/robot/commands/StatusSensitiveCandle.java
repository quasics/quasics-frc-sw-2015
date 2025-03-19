// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StatusSensitiveCandle extends Command {
  private final Candle m_candle;

  /** Creates a new StatusSensitiveCandle. */
  public StatusSensitiveCandle(Candle candle) {
    m_candle = candle;
    addRequirements(m_candle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Color8Bit color = Candle.GREEN;
    double intensity = 1.0;

    if (!DriverStation.isEnabled()) {
      intensity = 0.2;
    }

    var optAlliance = DriverStation.getAlliance();
    if (optAlliance.isEmpty()) {
      color = Candle.GREEN;
    } else {
      var alliance = optAlliance.get();
      color = switch (alliance) {
        case Blue -> Candle.BLUE;
        case Red -> Candle.RED;
      };
    }

    m_candle.setIntensity(intensity);
    m_candle.setColor(color);
  }
}
