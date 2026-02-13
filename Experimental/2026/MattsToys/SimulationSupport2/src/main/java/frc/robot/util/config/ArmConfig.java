package frc.robot.util.config;

/**
 * Single-joint arm configuration settings.
 *
 * @param pid         PID settings for the arm
 * @param feedForward feedforward settings for the arm
 */
public record ArmConfig(
    PIDConfig pid, ArmFeedForwardConfig feedForward) {
}