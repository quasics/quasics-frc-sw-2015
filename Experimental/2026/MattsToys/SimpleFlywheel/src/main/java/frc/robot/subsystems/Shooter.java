package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim; // REV-specific sim
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim; // WPILib physics sim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax motor;
  private final SparkClosedLoopController pidController;

  // Simulation Objects
  private SparkMaxSim motorSim;
  private FlywheelSim flywheelSim;

  public Shooter(int canID) {
    motor = new SparkMax(canID, MotorType.kBrushless);
    pidController = motor.getClosedLoopController();

    // Configure the motor controller
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.p(0.0005);
    // config.closedLoop.velocityFF(0.00017);
    config.closedLoop.feedForward.kV(0.00017);
    motor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize Simulation only if running on desktop
    if (RobotBase.isSimulation()) {
      // Define the physics of your shooter
      final DCMotor gearbox = DCMotor.getNEO(1); // Motor type and count
      final LinearSystem<N1, N1, N1> plant =
          LinearSystemId.createFlywheelSystem(gearbox,
              0.008, // Moment of Inertia (kg*m^2) - adjust to your wheel size);
              1.0 // Gearing (1:1),
          );

      flywheelSim = new FlywheelSim(plant, gearbox);
      motorSim = new SparkMaxSim(motor, gearbox); // Connect sim to motor
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Actual RPM", getActualRPM());
  }

  // Call this in the robot's simulationPeriodic() method
  @Override
  public void simulationPeriodic() {
    final double timeDeltaSecs = 0.020; // Standard 20ms loop

    // 1. Get applied voltage from the SparkMax
    double motorVoltage =
        motorSim.getAppliedOutput() * motorSim.getBusVoltage();

    // 2. Update physics model with that voltage
    flywheelSim.setInputVoltage(motorVoltage);
    flywheelSim.update(timeDeltaSecs);

    // 3. Feed the calculated velocity back into the SparkMax encoder
    motorSim.iterate(flywheelSim.getAngularVelocityRPM(),
        motorSim.getBusVoltage(), timeDeltaSecs);
  }

  public void setRPM(double targetRPM) {
    pidController.setSetpoint(targetRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
  }

  public double getActualRPM() {
    return motor.getEncoder().getVelocity();
  }
}
