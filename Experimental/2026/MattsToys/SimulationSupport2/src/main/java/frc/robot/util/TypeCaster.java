package frc.robot.util;

/**
 * Utility class with some casting-to-generics support.
 *
 * Sample use:
 *
 * <pre>
 * Object someObject = "Hello World";
 *
 * // Example of correct usage:
 * String castedString = castToGenericType(someObject, String.class);
 *
 * // Example of "incorrect" usage, which would throw an exception:
 * Integer castedInteger = castToGenericType(someObject, Integer.class);
 *
 * </pre>
 */
public class TypeCaster {
  /**
   * Safely casts an object to the type specified by the provided Class object.
   *
   * @param <T>    The target type.
   * @param object The object to cast.
   * @param type   The Class object representing the target type.
   * @return The casted object of type T.
   * @throws ClassCastException if the object is not compatible with the
   *     specified
   *                            type.
   */
  public static <T> T castToGenericType(Object object, Class<T> type) {
    // The Class.cast() method performs a runtime check and handles the casting.
    return type.cast(object);
  }

  /**
   * Safely casts an object to the type specified by the provided Class object.
   *
   * @param <T>    The target type.
   * @param object The object to cast.
   * @param type   The Class object representing the target type.
   * @return The casted object of type T, or null if the cast is unsafe.
   */
  public static <T> T castToGenericTypeOrNull(Object object, Class<T> type) {
    if (type.isInstance(object)) {
      return type.cast(object);
    }

    // Can't do the cast safely.
    return null;
  }
}
