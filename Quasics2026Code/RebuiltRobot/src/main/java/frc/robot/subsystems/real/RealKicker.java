// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.subsystems.SingleSparkMaxThing;
import frc.robot.subsystems.interfaces.IIndexer;
import frc.robot.subsystems.interfaces.IKicker;

public class RealKicker extends SingleSparkMaxThing implements IKicker {
  /** Creates a new RealIndexer. */
  public RealKicker() {
    super(SparkMaxIds.KICKER_ID);
  }

  @Override
  public void setKickSpeed(double speed) {
    setSpeed(speed);
  }

  @Override
  public void stopKicker() {
    stop();
  }

  @Override
  public double getKickerPosition() {
    // TODO Get Kicker position (rushing)
    throw new UnsupportedOperationException("Unimplemented method 'getKickerPosition'");
  }

}
