package frc.robot.subsystems.simulations;

/**
 * Defines the ports used for the simulation of the robot.
 */
public interface SimulationPorts {
  public final int LEFT_DRIVE_PWM_ID = 0;
  public final int RIGHT_DRIVE_PWM_ID = 1;

  public final int ELEVATOR_PWM_ID = 5;

  public final int GYRO_CHANNEL = 0;

  public final int LEFT_DRIVE_ENCODER_PORT_A = 0;
  public final int LEFT_DRIVE_ENCODER_PORT_B = 1;
  public final int RIGHT_DRIVE_ENCODER_PORT_A = 2;
  public final int RIGHT_DRIVE_ENCODER_PORT_B = 3;

  public final int ELEVATOR_ENCODER_PORT_A = 4;
  public final int ELEVATOR_ENCODER_PORT_B = 5;
}
