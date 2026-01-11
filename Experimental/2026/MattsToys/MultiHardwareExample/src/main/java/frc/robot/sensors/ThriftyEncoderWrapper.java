// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

// import com.thethriftybot.ThriftyNova;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ThriftyEncoderWrapper implements TrivialEncoder {

    public ThriftyEncoderWrapper() {

    }

    @Override
    public Distance getPosition() {
      return Meters.of(0);
    }

    @Override
    public LinearVelocity getVelocity() {
      return MetersPerSecond.of(0);
    }

    @Override
    public void reset() {
    }
    
}
