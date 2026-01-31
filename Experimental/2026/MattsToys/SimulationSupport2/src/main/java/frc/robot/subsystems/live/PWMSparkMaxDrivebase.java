// Copyright (c) 2026, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.live;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.constants.robots.SimulationPorts;
import frc.robot.hardware.actuators.IMotorControllerPlus;
import frc.robot.hardware.sensors.IGyro;
import frc.robot.hardware.sensors.TrivialEncoder;
import frc.robot.subsystems.DrivebaseBase;
import frc.robot.util.RobotConfigs.DriveConfig;

/**
 * Sample implementation of the drivebase functionality, based on PWMSparkMax
 * controllers.
 *
 * I'm using PWMSparkMax controllers in this code because of a bug in the
 * original releases of RevLib, which caused crashes during simulation, at least
 * under MacOS. (I filed a [bug
 * report](https://github.com/wpilibsuite/2026Beta/issues/29), and the issue has
 * been resolved.)
 *
 * Simulation support was provided in a subclass, SimDrivebase. This was done
 * simply to keep simulation-specific code separate from "real" robot code, in
 * order to provide greater clarity as an example; this functionality could
 * easily be merged into this class instead.
 */
public class PWMSparkMaxDrivebase extends DrivebaseBase {
  /** Creates a new Drivebase. */
  public PWMSparkMaxDrivebase(DriveConfig config) {
    super(config,
        IMotorControllerPlus.forPWMMotorController(
            new PWMSparkMax(SimulationPorts.PWM.LEFT_MOTOR_PORT)),
        IMotorControllerPlus.forPWMMotorController(
            new PWMSparkMax(SimulationPorts.PWM.RIGHT_MOTOR_PORT)),
        TrivialEncoder.forWpiLibEncoder(
            getConfiguredEncoder(SimulationPorts.DIO.LEFT_ENCODER_A_PORT,
                SimulationPorts.DIO.LEFT_ENCODER_B_PORT,
                config.orientation().isLeftInverted())),
        TrivialEncoder.forWpiLibEncoder(
            getConfiguredEncoder(SimulationPorts.DIO.RIGHT_ENCODER_A_PORT,
                SimulationPorts.DIO.RIGHT_ENCODER_B_PORT,
                config.orientation().isRightInverted())),
        IGyro.wrapGyro(new AnalogGyro(SimulationPorts.Channel.GYRO_PORT)),
        true);
  }
}
