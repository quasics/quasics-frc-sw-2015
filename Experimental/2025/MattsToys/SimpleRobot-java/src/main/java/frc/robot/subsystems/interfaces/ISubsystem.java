package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISubsystem {
    /** Convert the object to a subsystem (for listing in requirements). */
    default Subsystem asSubsystem() {
        return (Subsystem) this;
    }
}
