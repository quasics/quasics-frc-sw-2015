package frc.robot.util.config;

/**
 * Climber configuration settings.
 *
 * @param pid         PID settings for the climber
 * @param feedForward feedforward settings for the climber
 */
public record
    ClimberConfig(PIDConfig pid, SimpleFeedForwardConfig feedForward) {}