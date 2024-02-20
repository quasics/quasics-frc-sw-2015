// See discussions of unit testing in:
// * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
// * https://www.tutorialspoint.com/junit/index.htm

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
    final String stringKey = "key1";
    final String poseKey = "key2";
    final String uninsertedKey = "uninserted";

    board.updateValue(stringKey, "1");
    board.updateValue(
        poseKey,
        new Pose2d(
            Meters.of(1),
            Meters.of(1),
            new Rotation2d(Degrees.of(1))));

    assertEquals(
        Optional.of("1"),
        board.getValue(stringKey));
    assertEquals(
        Optional.of("1"),
        board.getValue(stringKey, String.class));
    assertEquals(
        Optional.empty(),
        board.getValue(stringKey, Pose2d.class));

    assertEquals(
        Optional.of(
            new Pose2d(
                Meters.of(1),
                Meters.of(1),
                new Rotation2d(Degrees.of(1)))),
        board.getValue(poseKey));
    assertEquals(
        Optional.of(
            new Pose2d(
                Meters.of(1),
                Meters.of(1),
                new Rotation2d(Degrees.of(1)))),
        board.getValue(poseKey, Pose2d.class));
    assertEquals(
        Optional.empty(),
        board.getValue(poseKey, String.class));

    // Putting something onto the board doesn't impact uninserted keys, right?
    assertEquals(
        /* expected */ Optional.empty(),
        /* actual */ board.getValue("uninserted"));
  }
}
