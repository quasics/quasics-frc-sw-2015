// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;

/** Support functions for working with ThriftyBot hardware. */
public class ThriftySupportFunctions {
  /**
   * Configures a motor (specified via CAN ID) to follow another motor. Also
   * copies over some key settings from the leader.
   *
   * @param followerId CAN ID for the motor to be configured as a follower
   * @param leader     the motor that should serve as leader
   *
   * @see
   *     https://docs.thethriftybot.com/thrifty-nova/software-resources/configure-controller-settings/follower-mode
   */
  public static void configureFollower(int followerId, ThriftyNova leader) {
    try (ThriftyNova follower = new ThriftyNova(followerId, MotorType.NEO)) {
      follower.follow(leader.getID());
      follower.setInverted(leader.getInversion());
    } catch (Exception e) { // Failures on ThriftyNova.close()
      throw new RuntimeException(e);
    }
  }
}
