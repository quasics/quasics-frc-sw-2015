package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISubsystem {
    default Subsystem asSubsystem() {
        return (Subsystem) this;
    }
}
