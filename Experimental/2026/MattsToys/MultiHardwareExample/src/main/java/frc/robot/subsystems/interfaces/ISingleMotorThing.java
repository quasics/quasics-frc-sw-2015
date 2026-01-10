package frc.robot.subsystems.interfaces;

import edu.wpi.first.units.measure.Distance;

public interface ISingleMotorThing {
    void setSpeed(double percent);
    double getSpeed();
    Distance getPosition();
    default void stop() { setSpeed(0); }
}
