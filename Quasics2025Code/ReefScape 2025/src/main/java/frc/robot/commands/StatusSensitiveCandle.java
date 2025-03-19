// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candle;

/**
 * Simple command to set the candle's color/intensity based on the alliance and
 * whether or not the robot is enabled.
 */
public class StatusSensitiveCandle extends Command {
  /** Subsystem being controlled. */
  private final Candle m_candle;

  /**
   * Constructor.
   * 
   * @param candle the candle subsystem being controlled
   */
  public StatusSensitiveCandle(Candle candle) {
    m_candle = candle;
    addRequirements(m_candle);
  }

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
