// See discussions of unit testing in:
// * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.utils.BulletinBoard;

public class BulletinBoardTest {
  BulletinBoard board;

  @BeforeEach // this method will run before each test
  void setup() {
    // Set up a fresh board every time.
    board = new BulletinBoard();
  }

  // @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() {
  }

  @Test
  void noDataWithoutInsertions() {
    assertEquals(
        /* expected */ Optional.empty(),
        /* actual */ board.getValue("uninserted"));
  }

  @Test
  void successfulRetrievalAfterInsertion() {
    final String key1 = "key1";
    final String key2 = "key2";
    final String uninsertedKey = "uninserted";

    board.updateValue(key1, "1");
    board.updateValue(
        key2,
        new Pose2d(
            Meters.of(1),
            Meters.of(1),
            new Rotation2d(Degrees.of(1))));

    assertEquals(
        Optional.of("1"),
        board.getValue(key1));
    assertEquals(
        Optional.of("1"),
        board.getValue(key1, String.class));
    assertEquals(
        Optional.empty(),
        board.getValue(key1, Pose2d.class));

    assertEquals(
        Optional.of(
            new Pose2d(
                Meters.of(1),
                Meters.of(1),
                new Rotation2d(Degrees.of(1)))),
        board.getValue(key2));
    assertEquals(
        Optional.of(
            new Pose2d(
                Meters.of(1),
                Meters.of(1),
                new Rotation2d(Degrees.of(1)))),
        board.getValue(key2, Pose2d.class));
    assertEquals(
        Optional.empty(),
        board.getValue(key2, String.class));

    // Putting something onto the board doesn't impact uninserted keys, right?
    assertEquals(
        /* expected */ Optional.empty(),
        /* actual */ board.getValue("uninserted"));
  }
}
