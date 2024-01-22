// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

/** Add your docs here. */
public class Constants {
  public class AprilTags {
    /** April tags are square images, 8 1/8 inch to a side. */
    public static final Measure<Distance> APRIL_TAG_SIZE = Inches.of(8).plus(Inches.of(1.0 / 8.0));

    /**
     * Height to the bottom of the AprilTags (1, 2, 9, 10) on the Source positions.
     */
    public static final Measure<Distance> SOURCE_TAG_BOTTOM_HEIGHT = Feet.of(4).plus(Inches.of(1.0 / 8.0));

    /** Height to the bottom of the AprilTags (3, 4, 7, 8) on the Speakers. */
    public static final Measure<Distance> SPEAKER_TAG_BOTTOM_HEIGHT = Feet.of(4).plus(Inches.of(3 + 7.0 / 8.0));

    /** Height to the bottom of the AprilTags (5, 6) on the Amps. */
    public static final Measure<Distance> AMP_TAG_BOTTOM_HEIGHT = Feet.of(4).plus(Inches.of(1.0 / 8.0));

    /** Height to the bottom of the AprilTags (11-16) on the Stages. */
    public static final Measure<Distance> STAGE_TAG_BOTTOM_HEIGHT = Feet.of(3).plus(Inches.of(11.5));
  }
}
