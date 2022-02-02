// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple design for control of the 2022 climber hardware, which is expected to
 * be built using "ThriftyBot extending climber kits".
 * 
 * Under this approach, a pair of retractable "arms" are used", which are pulled
 * down against a set of springs (when not climbing) and/or the weight of the
 * robot (when hanging) by a cable that is driven by a winch (controlled by this
 * subsystem).
 * 
 * @see https://www.thethriftybot.com/bearings/Round-2-Pre-Order-Thrifty-Telescoping-Tube-Kit-p416413760
 */
public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {
  }

  public void holdPosition() {
    // TODO: Implement this method.

    /*
     * [Notes from discussion among coaches on 28Jan:]
     * 
     * I think that something we'll also need to look at is how we're going to hold
     * ourselves aloft. Will we need to keep the motor running, or will we work with
     * a motor that can be put into a "brake" mode (like the SIMS)?
     * 
     * In the former case, we'd have to worry about burning the motor out unless
     * it's sufficiently "beefy". But if it *is* that beefy, then we might need to
     * worry about power draw (holding up against a 100+ pound robot, or wherever we
     * come in at) and/or whether we're going to have to play some games to keep it
     * from retracting too far and punching through something at the bottom (though
     * this would hopefully be something we can design against).
     * 
     * In the latter case, we'd need to make sure that the "brake" mode could hold
     * up against the pull of our weight.
     * 
     * Either option will have some implications for s/w.
     * 
     * It's worth noting that Nike was able to stay up without the ratchet, with a
     * 50:1 gearbox. But she was lighter than what Sally is expected to weigh by at
     * least a bit.
     */
  }

  public void resetPosition() {

  }

  public int getPosition() {
    return 0;
  }

  public void extendArms() {
    // TODO: Implement this method.
  }

  public void retractArms() {
    // TODO: Implement this method.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
