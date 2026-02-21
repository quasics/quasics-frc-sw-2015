// Copyright (c) 2025-2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.config;

/**
 * Configuration data for an elevator.
 *
 * @param pid         PID configuration settings for the elevator's motors
 * @param feedForward feedforward data for the elevator
 */
public record
    ElevatorConfig(PIDConfig pid, ElevatorFeedForwardConfig feedForward) {}