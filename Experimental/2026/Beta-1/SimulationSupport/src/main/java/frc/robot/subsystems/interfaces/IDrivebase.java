package frc.robot.subsystems.interfaces;

public interface IDrivebase extends ISubsystem {
    void driveArcade(double forward, double rotation);
    void tankDrive(double leftSpeed, double rightSpeed);
}
