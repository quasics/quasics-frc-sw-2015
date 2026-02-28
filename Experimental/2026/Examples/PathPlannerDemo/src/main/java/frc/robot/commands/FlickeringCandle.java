// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.ICandle;
import frc.robot.subsystems.interfaces.ILighting.StockColor;

/**
 * Simple command that turns the ICandle on and off.
 */
public class FlickeringCandle extends Command {
  /** The ICandle object being controlled. */
  final ICandle m_candle;

  /** Timer used to control changes. */
  Timer m_timer = new Timer();

  /** Flag indicating whether the candle is currently on. */
  boolean m_isOn = false;

  /** Period of time between visible state changes. */
  static final double FLICKER_PERIOD = 0.5;

  /**
   * Creates a new FlickeringCandle.
   * 
   * @param candle the ICandle object to control
   */
  public FlickeringCandle(ICandle candle) {
    m_candle = candle;
    addRequirements(candle.asSubsystem());
  }

  @Override
  public void initialize() {
    m_isOn = true;
    m_timer.reset();
    m_timer.start();
  }

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

  @Override
  public void end(boolean interrupted) {
    if (m_candle != null) {
      m_candle.setColor(StockColor.White);
    }
  }
}
