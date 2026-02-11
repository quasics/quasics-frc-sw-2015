// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.interfaces;

/**
 * Interface for controlling the shooter.
 * 
 * Per email from Daniel on 04Feb, the shooter characteristics are as follows.
 * 
 * * 1:1 Kraken X60 running flywheels: requires the ability to spin in one
 * direction; will need to be adjusted with PID control to maintain constant RPM
 * from ball to ball; around 7.5 m/s for a shot from the tower (I’m assuming
 * this just requires trial and error to define what % power the motor needs to
 * ideally be); a shot up in front of the hub is another shooting configuration
 * (not much data or testing on this yet)
 * 
 * * Neo 2.0 running kicker wheels: requires the ability to spin in one
 * direction; not sure if PID control is necessary or not due to my lack of
 * experience with it; gear reduction isn’t decided yet — it might not be needed
 * 
 * * 3:1 gearbox (or possibly greater) Neo 550 running hood angle adjustment
 * mechanism: requires the ability to spin in both directions; hood mechanism is
 * hard stopped (45-75 degree angle range); motor shaft belted to an axle with
 * 18t tooth gears meshing with the hood plates with teeth; gears cause an
 * additional 18:1 ish reduction; around 60 degree hood angle for a shot from
 * tower, which I think should be the default setting for shooting; probably
 * closer to 45 degree hood angle for feeding; probably closer to 75 degree hood
 * angle for shooting at the hub (I would assume these will be more definitively
 * defined after some further trial and error)
 * 
 * TODO: Define the shooter interface (and then implement it).
 */
public interface IShooter {

}
