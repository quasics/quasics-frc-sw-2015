// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.ILighting.StockColor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlickeringCandle extends Command {
  final ICandle m_candle;
  Timer m_timer = new Timer();
  boolean m_isOn = false;
  static final double FLICKER_PERIOD = 0.5;

  /** Creates a new FlickeringCandle. */
  public FlickeringCandle(ICandle candle) {
    m_candle = candle;
    addRequirements(candle.asSubsystem());
  }

  public void initialize() {
    m_isOn = true;
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_candle == null) {
      return;
    }

    if (m_timer.hasElapsed(FLICKER_PERIOD)) {
      m_isOn = !m_isOn;
      m_timer.reset();
    }

    if (m_isOn) {
      m_candle.setColor(StockColor.Green);
    } else {
      m_candle.setColor(StockColor.Black);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_candle != null) {
      m_candle.setColor(StockColor.White);
    }
  }
}
