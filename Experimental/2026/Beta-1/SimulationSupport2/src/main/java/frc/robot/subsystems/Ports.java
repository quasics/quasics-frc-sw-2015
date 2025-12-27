// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public interface Ports {
  // PWM Ports
  public static final int LEFT_MOTOR_PWM_PORT = 1;
  public static final int RIGHT_MOTOR_PWM_PORT = 2;

  // DIO Ports
  public static final int LEFT_ENCODER_A_DIO_PORT = 0;
  public static final int LEFT_ENCODER_B_DIO_PORT = 1;
  public static final int RIGHT_ENCODER_A_DIO_PORT = 2;
  public static final int RIGHT_ENCODER_B_DIO_PORT = 3;

  // Channel Ports
  public static final int GYRO_CHANNEL_PORT = 0;
}
