// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.util.SpeedMode;
import frc.robot.util.SpeedModeScaler;

/**
 * Provides basic unit testing for the SpeedModeScaler class.
 * 
 * @see frc.robot.util.SpeedModeScaler
 */
public class SpeedModeScalerTest {

    /** Scaler instance being used for testing. */
    SpeedModeScaler speedModeScaler;

    /** Basic value for "turtle" scaling. */
    static final double TURTLE_SCALING = 0.50;
    /** Basic value for "normal" scaling. */
    static final double NORMAL_SCALING = 0.65;
    /** Basic value for "turbo" scaling. */
    static final double TURBO_SCALING = 0.80;
    /** Acceptable deviation range when testing scaling outputs. */
    static final double DELTA = 1e-2;

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

    @Test
    void constructionTests() {
        //
        // Invalid constructions
        //

        // SpeedMode supplier is null (invalid)
        assertThrows(IllegalArgumentException.class, () -> {
            new SpeedModeScaler(null /* invalid! */, NORMAL_SCALING, TURBO_SCALING, TURTLE_SCALING);
        });

        // Normal and turbo scaling are swapped
        assertThrows(IllegalArgumentException.class, () -> {
            new SpeedModeScaler(() -> SpeedMode.Normal, TURBO_SCALING, NORMAL_SCALING, TURTLE_SCALING);
        });

        // Normal and turtle scaling are swapped
        assertThrows(IllegalArgumentException.class, () -> {
            new SpeedModeScaler(() -> SpeedMode.Normal, TURTLE_SCALING, TURBO_SCALING, NORMAL_SCALING);
        });

        //
        // These should *not* throw, as they're potentially valid
        //

        // All scaling factors the same (so, all scaling is disabled)
        new SpeedModeScaler(() -> SpeedMode.Normal, NORMAL_SCALING, NORMAL_SCALING, NORMAL_SCALING);

        // Turbo and normal scaling the same (i.e., turbo mode is disabled)
        new SpeedModeScaler(() -> SpeedMode.Normal, NORMAL_SCALING, NORMAL_SCALING, TURTLE_SCALING);

        // Turtle and normal scaling the same (i.e., turtle mode is disabled)
        new SpeedModeScaler(() -> SpeedMode.Normal, NORMAL_SCALING, TURBO_SCALING, NORMAL_SCALING);
    }
}
