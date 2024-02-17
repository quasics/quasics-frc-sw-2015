// Copyright (c) 2024, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/** Add your docs here. */
public class BulletinBoard {
  private static Map<String, Object> values = new HashMap<String, Object>();

  public static synchronized Optional<Object> getValue(String key) {
    if (!values.containsKey(key)) {
      return Optional.empty();
    }
    return Optional.of(values.get(key));
  }

  public static synchronized Optional<Object> getValue(String key, Class type) {
    if (!values.containsKey(key)) {
      return Optional.empty();
    }

    Object o = values.get(key);
    if (type != null && !type.isInstance(o)) {
      return Optional.empty();
    }
    return Optional.of(values.get(key));
  }

  public static synchronized void clearValue(String key) {
    values.remove(key);
  }

  public static synchronized void updateValue(String key, Object value) {
    values.put(key, value);
  }
}
