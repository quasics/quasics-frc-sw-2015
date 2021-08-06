// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Relay;

public class ExampleSubsystem extends SubsystemBase {
  private Relay relay;

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    relay = new Relay(Constants.RELAY_SLOT);
    relayOff();
  }

  public void relayOn() {
    System.out.println("Relay on");
    relay.set(Relay.Value.kOn);
  }

  public void relayForward() {
    System.out.println("Relay forward");
    relay.set(Relay.Value.kForward);
  }

  public void relayReverse() {
    System.out.println("Relay reverse");
    relay.set(Relay.Value.kReverse);
  }

  public void relayOff() {
    System.out.println("Relay off");
    relay.set(Relay.Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
