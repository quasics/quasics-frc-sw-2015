// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import frc.robot.Constants;

/**
 * Defines the ports used for the simulation of the robot.
 */
public interface SimulationPorts {
  /** PWM port used for left motor. */
  public final int LEFT_DRIVE_PWM_ID = 0;

  /** PWM port used for right motor. */
  public final int RIGHT_DRIVE_PWM_ID = 1;

  /** PWM port used for lighting control. */
  public final int LIGHTING_PWM_ID = 2;

  /** CAN ID used for elevator motor. */
  public final int ELEVATOR_CAN_ID = 1;

  /** CAN ID used for single-joint arm's motor. */
  public final int ARM_CAN_ID = 2;

  /** Channel ID used for the gyro. */
  public final int GYRO_CHANNEL = 0;

  /** "A" port used for the encoder on the left side of the drivebase. */
  public final int LEFT_DRIVE_ENCODER_PORT_A = Constants.DioIds.LAST_NONSIM_DIO + 1;
  /** "B" port used for the encoder on the left side of the drivebase. */
  public final int LEFT_DRIVE_ENCODER_PORT_B = LEFT_DRIVE_ENCODER_PORT_A + 1;
  /** "A" port used for the encoder on the right side of the drivebase. */
  public final int RIGHT_DRIVE_ENCODER_PORT_A = LEFT_DRIVE_ENCODER_PORT_B + 1;
  /** "B" port used for the encoder on the right side of the drivebase. */
  public final int RIGHT_DRIVE_ENCODER_PORT_B = RIGHT_DRIVE_ENCODER_PORT_A + 1;
}
