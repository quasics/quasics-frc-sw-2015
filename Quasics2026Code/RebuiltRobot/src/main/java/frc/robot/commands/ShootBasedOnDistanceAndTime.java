package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.interfaces.IDrivebase;
import frc.robot.subsystems.interfaces.IShooter;

public class ShootBasedOnDistanceAndTime extends ShootBasedOnDistance {
  private final Timer m_timer = new Timer();
  private final Time m_duration;

  public ShootBasedOnDistanceAndTime(IShooter shooter, IDrivebase drivebase, double kickSpeed, double kickerDelay,
      Time duration) {
    super(shooter, drivebase, kickSpeed, kickerDelay);
    m_duration = duration;
  }

  @Override
  public void initialize() {
    super.initialize();
    m_timer.restart();
  }

  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_duration)) {
      return true;
    }
    return super.isFinished();
  }
}
