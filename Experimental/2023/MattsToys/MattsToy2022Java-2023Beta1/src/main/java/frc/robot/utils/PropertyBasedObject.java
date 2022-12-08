package frc.robot.utils;

import java.util.Properties;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class PropertyBasedObject {
  /**
   * Builds a Properties object containing all of the data represented by the
   * provided object, with the keys being the names of each data field, and the
   * values being a string version of the coresponding field value.
   * 
   * @return a Properties object holding key/value pairs for all of this object's
   *         data
   * @throws IllegalArgumentException
   * @throws IllegalAccessException
   */
  public Properties buildProperties() throws IllegalArgumentException, IllegalAccessException {
    Properties props = new Properties();
    Class<?> myClass = getClass();
    Field[] fields = myClass.getFields();

    for (var field : fields) {
      if (Modifier.isPublic(field.getModifiers())) {
        props.setProperty(field.getName(), field.get(this).toString());
      }
    }

    return props;
  }

  @Override
  public String toString() {
    StringBuilder sb = new StringBuilder(getClass().getCanonicalName());
    sb.append(":");
    try {
      sb.append(buildProperties().toString());
    } catch (Exception e) {
      sb.append("<Error: Failed to extract data for property based object>");
    }
    return sb.toString();
  }

  @Override
  public boolean equals(Object o) {
    if (o == null) {
      return false;
    }
    if (!(o instanceof PropertyBasedObject)) {
      return false;
    }

    PropertyBasedObject other = (PropertyBasedObject) o;
    try {
      return this.buildProperties().equals(other.buildProperties());
    } catch (IllegalArgumentException | IllegalAccessException e) {
      return false;
    }
  }
}
