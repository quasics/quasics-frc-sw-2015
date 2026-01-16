import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.util.SpeedMode;
import frc.robot.util.SpeedModeScaler;

public class SpeedModeScalerTest {

    SpeedModeScaler speedModeScaler;
    static final double TURTLE_SCALING = 0.50;
    static final double NORMAL_SCALING = 0.65;
    static final double TURBO_SCALING = 0.80;
    static final double DELTA = 1e-2; // acceptable deviation range

    @BeforeEach
    void setup() {
        // Do any common pre-test setup in here (e.g., creating a subsystem,
        // initializing the HAL, etc.)
    }

    @AfterEach
    void shutdown() throws Exception {
        // Do any common post-test cleanup in here (e.g., calling "close()" on subsystem
        // objects)
    }

    @Test
    void normalScalingTest() {
        speedModeScaler = new SpeedModeScaler(() -> SpeedMode.Normal, NORMAL_SCALING, TURBO_SCALING, TURTLE_SCALING);
        assertEquals(NORMAL_SCALING * 0, speedModeScaler.apply(0.0), DELTA);
        assertEquals(NORMAL_SCALING * +.5, speedModeScaler.apply(+.5), DELTA);
        assertEquals(NORMAL_SCALING * -.5, speedModeScaler.apply(-.5), DELTA);
    }

    @Test
    void turboScalingTest() {
        speedModeScaler = new SpeedModeScaler(() -> SpeedMode.Turbo, NORMAL_SCALING, TURBO_SCALING, TURTLE_SCALING);
        assertEquals(TURBO_SCALING * 0, speedModeScaler.apply(0.0), DELTA);
        assertEquals(TURBO_SCALING * +.5, speedModeScaler.apply(+.5), DELTA);
        assertEquals(TURBO_SCALING * -.5, speedModeScaler.apply(-.5), DELTA);
    }

    @Test
    void turtleScalingTest() {
        speedModeScaler = new SpeedModeScaler(() -> SpeedMode.Turtle, NORMAL_SCALING, TURBO_SCALING, TURTLE_SCALING);
        assertEquals(TURTLE_SCALING * 0, speedModeScaler.apply(0.0), DELTA);
        assertEquals(TURTLE_SCALING * +.5, speedModeScaler.apply(+.5), DELTA);
        assertEquals(TURTLE_SCALING * -.5, speedModeScaler.apply(-.5), DELTA);
    }
}
