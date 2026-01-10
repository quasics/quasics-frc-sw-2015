// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants.robots;

/** Class defining the ports/channels used by the robot subsystems. */
public class SimulationPorts {
  /** PWM Port allocations. */
  public class PWM {
    public static final int LEFT_MOTOR_PORT = 1;
    public static final int RIGHT_MOTOR_PORT = 2;
    public static final int LIGHTING_PORT = 3;
  }

  /** DIO Port allocations. */
  public class DIO {
    public static final int LEFT_ENCODER_A_PORT = 0;
    public static final int LEFT_ENCODER_B_PORT = 1;
    public static final int RIGHT_ENCODER_A_PORT = 2;
    public static final int RIGHT_ENCODER_B_PORT = 3;
  }

  /** Channel allocations. */
  public class Channel {
    public static final int GYRO_PORT = 0;
  }
}
