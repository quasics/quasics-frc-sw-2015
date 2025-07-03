// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.IVision;
import java.util.List;

public class SimpleVision extends SubsystemBase implements IVision {
  /** Creates a new SimpleVision. */
  public SimpleVision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public boolean hasTargetsInView() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'hasTargetsInView'");
  }

  @Override
  public List<TargetData> getTargets() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTargets'");
  }
}
