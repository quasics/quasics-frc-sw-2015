// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivebase.AbstractDrivebase;

/**
 * Utility class to support generating commands used to gather data for SysId
 * characterization of a robot.
 */
public class SysIdGenerator {
    /** Profiling modes. */
    public enum Mode {
        /** Profiling linear movement. */
        Linear,
        /** Profiling rotational movement. */
        Rotating
    }

    final static boolean DUMP_SYSID_TO_CONSOLE = true;

    /**
     * Returns a SysIdRoutine generator for the specified drivebase/mode.
     *
     * @param drivebase drive base of the robot being characterized
     * @param mode      movement mode being characterized
     * @return a configured SysIdRoutine generator
     */
    public static SysIdRoutine getSysIdRoutine(final AbstractDrivebase drivebase, final Mode mode) {
        return new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step
                // voltage.
                new SysIdRoutine.Config(Volts.of(0.5).div(Seconds.of(1)), Volts.of(3.5), Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            drivebase.setMotorVoltages(volts, volts.times(mode == Mode.Linear ? 1 : -1));
                        },
                        // Tell SysId how to record a frame of data for each motor on the
                        // mechanism being characterized.
                        log -> {
                            final var leftPosition = drivebase.getLeftPosition();
                            final var leftVelocity = drivebase.getLeftVelocity();
                            final var leftVoltage = drivebase.getLeftVoltage();
                            final var rightPosition = drivebase.getRightPosition();
                            final var rightVelocity = drivebase.getRightVelocity();
                            final var rightVoltage = drivebase.getRightVoltage();

                            if (DUMP_SYSID_TO_CONSOLE) {
                                System.err.println("Logging "
                                        + "left=" + String.format("%,.3f", leftVoltage.in(Volts)) + "V, "
                                        + String.format("%,.3f", leftPosition.in(Meters)) + "m, "
                                        + String.format("%,.3f", leftVelocity.in(MetersPerSecond)) + "m/s   "
                                        + "right=" + String.format("%,.3f", rightVoltage.in(Volts)) + "V, "
                                        + String.format("%,.3f", rightPosition.in(Meters)) + "m, "
                                        + String.format("%,.3f", rightVelocity.in(MetersPerSecond)) + "m/s   ");
                            }

                            // Record a frame for the left motors. Since these share an encoder,
                            // we consider the entire group to be one motor.
                            log.motor("drive-left")
                                    .voltage(leftVoltage)
                                    .linearPosition(leftPosition)
                                    .linearVelocity(leftVelocity);
                            // Record a frame for the right motors. Since these share an
                            // encoder, we consider the entire group to be one motor.
                            log.motor("drive-right")
                                    .voltage(rightVoltage)
                                    .linearPosition(rightPosition)
                                    .linearVelocity(rightVelocity);
                        },
                        drivebase));
    }

    /**
     * @param drivebase drive base of the robot being characterized
     * @param mode      movement mode being characterized
     * @param direction direction of movement being characterized
     * @return a Command for use in running quasistatic profiling in the
     *         specified direction.
     */
    public static Command sysIdQuasistatic(
            AbstractDrivebase drivebase, Mode mode, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(drivebase, mode).quasistatic(direction);
    }

    /**
     * @param drivebase drive base of the robot being characterized
     * @param mode      movement mode being characterized
     * @param direction direction of movement being characterized
     * @return a Command for use in running dynamic profiling in the
     *         specified direction.
     */
    public static Command sysIdDynamic(
            AbstractDrivebase drivebase, Mode mode, SysIdRoutine.Direction direction) {
        return getSysIdRoutine(drivebase, mode).dynamic(direction);
    }
}
