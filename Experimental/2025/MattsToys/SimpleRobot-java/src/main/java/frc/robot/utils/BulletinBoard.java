// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * This class provides a simple means for "publishing" data from a given
 * component, for use in other places that are otherwise "disconnected".
 *
 * For example, a vision subsystem might compute an approximate position of the
 * robot (based on camera information) that can be used by the drive base to
 * improve its calculation of the robot's pose. Similarly, the vision subsystem
 * might use the most recent location computed by the drive base (via odometry
 * or other means) as a reference position for vision-based calculations. Rather
 * than one subsystem having to "know about" another (and thus providing direct
 * cyclic dependencies), they can simply post updates to this shared location
 * for client access.
 *
 * Another alternative might be a more formal "pub/sub" (publish/subscribe)
 * pattern, including notifications on changes, but that feels like overkill for
 * my current needs, and this would still require at least some degree of direct
 * cyclic dependency between peer components. Similar concerns would exist for
 * using the "Observer" pattern, etc., and the standard Java "Observable" type
 * is also a concrete class (rather than an adaptable interface), which makes it
 * hard to use trivially with WPILib subsystems.
 *
 * @see https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern
 * @see https://en.wikipedia.org/wiki/Observer_pattern
 *
 */
public class BulletinBoard {
  /**
   * Default (singleton) instance of the class.
   */
  public static final BulletinBoard common = new BulletinBoard();

  private Map<String, Object> values = new HashMap<String, Object>();

  /**
   * Retrieves the value associated with the specified key (if any is present),
   * without any typing constraints being applied.
   */
  public synchronized Optional<Object> getValue(String key) {
    if (!values.containsKey(key)) {
      return Optional.empty();
    }
    return Optional.of(values.get(key));
  }

  /**
   * Retrieves the value associated with the specified key (if any is present, and
   * if it is compliant with the specified type).
   */
  public synchronized Optional<Object> getValue(String key, Class<?> type) {
    if (!values.containsKey(key)) {
      return Optional.empty();
    }

    Object o = values.get(key);
    if (type != null && !type.isInstance(o)) {
      return Optional.empty();
    }
    return Optional.of(values.get(key));
  }

  /**
   * Removes any previously-set value associated with the specified key.
   */
  public synchronized void clearValue(String key) {
    values.remove(key);
  }

  /**
   * Sets/updates the value associated with the specified key, allowing later
   * retrieval with "getValue()".
   */
  public synchronized void updateValue(String key, Object value) {
    values.put(key, value);
  }
}
