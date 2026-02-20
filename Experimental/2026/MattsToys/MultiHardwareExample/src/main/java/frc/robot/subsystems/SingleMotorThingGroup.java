// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.interfaces.ISingleMotorThing;

/**
 * A simple implementation of ISingleMotorThing that groups multiple
 * ISingleMotorThing instances together.
 * 
 * This is useful for testing or for creating a "virtual" single motor thing
 * that controls multiple underlying things at once.
 * 
 * For example, you could use this to create a "virtual" single motor thing that
 * controls both a left and right motor together, treating them as one unit.
 * 
 * Note that the getPosition() method just returns the position of the first
 * thing for simplicity, so this is really only intended for testing or for
 * cases where the position of the first thing is representative of the whole
 * group.
 */
public class SingleMotorThingGroup implements ISingleMotorThing {
  private final List<ISingleMotorThing> m_things;

  /**
   * Creates a new SingleMotorThingGroup with the given things.
   * 
   * @param things
   */
  public SingleMotorThingGroup(ISingleMotorThing... things) {
    m_things = List.of(things);
    if (m_things.isEmpty()) {
      throw new IllegalArgumentException("Must have at least one SingleMotorThing");
    }
  }

  /**
   * Adds another thing to this group. (This is not thread-safe, so it should only
   * be called during initialization.)
   *
   * @param thing the thing to add
   */
  public void addThing(ISingleMotorThing thing) {
    m_things.add(thing);
  }

  @Override
  public void setSpeed(double percent) {
    for (ISingleMotorThing thing : m_things) {
      thing.setSpeed(percent);
    }
  }

  @Override
  public double getSpeed() {
    return m_things.stream().mapToDouble(ISingleMotorThing::getSpeed).average().orElse(0.0);
  }

  @Override
  public Distance getPosition() {
    return m_things.get(0).getPosition(); // Just return the position of the first thing for simplicity.
  }
}
