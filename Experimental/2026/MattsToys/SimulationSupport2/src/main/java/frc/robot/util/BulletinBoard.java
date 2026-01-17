// Copyright (c) 2024-2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * This class provides a simple means for "publishing" data from a given
 * component, for use in other places that are otherwise "disconnected".
 *
 * The main idea here is that it's often considered to be a bad idea for a
 * specific object to "know about" (i.e., have a direct reference to) some
 * other (generally unrelated) object, but there might need to be some actual
 * information-sharing between them for specificic reasons.
 *
 * As a concrete example:
 * <ul>
 * <li>
 * A vision subsystem might compute an approximate position of the
 * robot (based on camera information) that can be used by the drive base to
 * improve its calculation of the robot's pose.
 * </li>
 * <li>
 * Similarly, the vision subsystem might use the most recent location computed
 * by the drive base (via odometry or other means) as a reference position for
 * vision-based calculations.
 * </li>
 * <li>
 * But subsystems are usually supposed to be isolated from each other, and for
 * good reason.  For example, we might be using a "naked" drive base (without
 * any other hardware) for initial code bring-up, driver practice, or coder
 * training, and thus might not create a vision subsystem (no camera!).
 * </li>
 * </ul>
 *
 * This class lets any piece of code "post a notice" with information that other
 * parts of the program may (or may not) care about in a common place.  When
 * some other code wants to use that information, they can "look at the board"
 * to see if it's available, without needing to have a direct connection to
 * whatever object supplied it.
 *
 * (The following is more detail about this general idea, and some options.
 * Feel free to ignore it, and just focus on using this class if you want.)
 *
 * The "publish–subscribe" model (also sometimes called "pub/sub") is a design
 * pattern sometimes used in software development for this sort of purpose.  In
 * this model, message senders ("publishers") can organize information (e.g.,
 * changes in their state, external readings from sensors, etc.) into categories
 * (classes, topics, etc.) of messages, and send them without needing to know
 * what other pieces of code will work with this data. Message recipients
 * ("subscribers"), express interest in one or more categories and only receive
 * messages of that type, without needing to know the identity of the
 * publishers.
 *
 * This often involves a message "broker" (like the US post office) to which the
 * publishers send their updates, and then the broker actively relays copies of
 * the data to any registered subscriber objects (e.g., by calling a
 * "processUpdate(data....)" method on them).  Some industrial-level examples
 * of brokers include the
 * <a href="https://en.wikipedia.org/wiki/Java_Message_Service">Java Message
 * Service</a> and <a href="https://en.wikipedia.org/wiki/MQTT">MQTT</a>.
 *
 * The full "pub/sub" model feels like a bit of overkill for my current needs.
 *
 * For one thing, I don't want to have a full message broker sitting in the
 * middle (either one that I pull off of a shelf, or something else that I
 * implement).
 *
 * For another, given the timing constraints on different pieces of code on an
 * FRC bot, it could be tough to figure out when it's "safe" for it to try to
 * send data out to everything that's interested. After all, commands,
 * periodic() functions, etc. are supposed to run *fast* and get out of the way
 * of other stuff; knowing when to do that could be hard.  (And while I could
 * handle a lot of this risk by introducing additional threads of execution and
 * synchronization/atomics, that involves another set of stuff that FRC
 * programs generally avoid by having a single primary thread of execution
 * where the WPILib code "does its thing".)
 *
 * I also considered using the "Observer" pattern, etc., but that requires that
 * something (e.g., the RobotContainer class) actively work to establish real
 * connections between the observers (effectively subscribers) and the observed
 * object (effectively publishers), which breaks the pure isolation for
 * subsystems. And the standard Java "Observable" type is also a concrete class
 * that a Subsystem can't derive from, rather than being an adaptable interface,
 * which makes it hard to use Java's default implementation of this pattern
 * trivially with WPILib subsystems.
 *
 * So, using a "bulletin board" model for data-sharing seemed like a reasonable
 * approach: I still have something that serves as a *kind* of a message
 * broker, but it's a passive part of the system ("Tell me if you want to share
 * some information", "Ask me if some data is available").  As long as the
 * clients (subscribers) handle the case where data isn't available, then
 * there's very little overhead, and the subsystems have no direct connections
 * to each other.
 *
 * @see <a href=
 *      "https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern">Publish/subscribe
 *      pattern</a>
 * @see <a href="https://en.wikipedia.org/wiki/Observer_pattern">Observer
 *      pattern"</a>
 * @see java.util.Observable
 * @see java.util.Observer
 */
public class BulletinBoard {
  /**
   * Default (singleton) instance of the class.
   */
  public static final BulletinBoard common = new BulletinBoard();

  /** Maps stored values to their associated keys. */
  private Map<String, Object> values = new HashMap<String, Object>();

  /**
   * Indicates if there is any data associated with the specified key.
   *
   * @param key the key being checked
   * @return true iff the key is populated
   *
   * @see #updateValue(String, Object)
   */
  public synchronized boolean hasKey(String key) {
    return values.containsKey(key);
  }

  /**
   * Indicates if there is any data associated with the specified key, and
   * removes it from the data set. (Useful for one subsystem asynchronously
   * requesting an operation of another.)
   *
   * @param key the key being checked
   * @return true iff the key was populated
   *
   * @see #updateValue(String, Object)
   */
  public synchronized boolean testAndClear(String key) {
    boolean result = values.containsKey(key);
    if (result) {
      values.remove(key);
    }
    return result;
  }

  /**
   * Retrieves the value associated with the specified key (if any is present),
   * without any typing constraints being applied.
   *
   * @param key the key for the desired value
   * @return an Optional wrapping the desired value, if found
   *
   * @see #updateValue(String, Object)
   */
  public synchronized Optional<Object> getValue(String key) {
    if (!values.containsKey(key)) {
      return Optional.empty();
    }
    return Optional.of(values.get(key));
  }

  /**
   * Retrieves the value associated with the specified key (if any is present,
   * and if it is compliant with the specified type).
   *
   * @param key  the key for the desired value
   * @param type the type of data expected for the value
   * @return an Optional wrapping the desired value, if a match is found
   *
   * @see #updateValue(String, Object)
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
   *
   * @param key the key for the targeted value
   */
  public synchronized void clearValue(String key) {
    values.remove(key);
  }

  /**
   * Sets/updates the value associated with the specified key, allowing later
   * retrieval with "getValue()".
   *
   * @param key   the key associated with the value being stored
   * @param value the value to be stored
   *
   * @see #getValue(String)
   * @see #getValue(String, Class)
   */
  public synchronized void updateValue(String key, Object value) {
    values.put(key, value);
  }
}
