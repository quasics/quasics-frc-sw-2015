// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public interface ISingleJointArm extends ISubsystem {
  ////////////////////////////////////////////////////////////////////////////////////
  // Values defining the arm's characteristics/physics
  ////////////////////////////////////////////////////////////////////////////////////

  final Angle ARM_OUT_ANGLE = Degrees.of(180);
  final Angle ARM_UP_ANGLE = Degrees.of(90);
  final double GEARING = 5 * 5 * 3 * 4.44; // Arbitrary (but needs to be enough for
                                           // simulated physics to work)
  final double ARM_LENGTH_METERS = 1.0; // Arbitrary
  final double ARM_MASS_KG = 4.0; // Arbitrary

  String SUBSYSTEM_NAME = "Arm";

  void setTargetPosition(Angle targetPosition);

  final ISingleJointArm NULL_ARM = new ISingleJointArm() {
    @Override
    public void setTargetPosition(Angle targetPosition) {
      System.out.println("Null arm: setting target position to " + targetPosition);
      return;
    }
  };
}
