// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AbstractElevator;

public class SimulatedElevator extends AbstractElevator {
  static final double EXTENSION_SPEED = +1.0;
  static final double RETRACTION_SPEED = -1.0;

  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_gearing = DCMotor.getNEO(1);

  private final SparkMax m_motor = new SparkMax(SimulationPorts.ELEVATOR_CAN_ID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  // Mechanism2d visualization of the hardware (for rendering in
  // SmartDashboard, or the simulator).
  private final MechanismLigament2d m_mech2d;

  //////////////////////////////////////////////////////////////////////////////
  // Simulation support data/objects

  /** Motor being driven by the controller. */
  private DCMotor elevatorPlant = DCMotor.getNEO(1);

  // Simulation motors/encoders
  private final SparkMaxSim m_motorSim = new SparkMaxSim(m_motor, elevatorPlant);

  // TODO: Update these constants to better emulate the real behavior of the
  // hardware. (But for now, this will at least give us something we can use.)
  private static final double kGearing = 10.0;
  private static final Distance kDrumRadius = Units.Inches.of(1);
  private static final double kEncoderMetersPerPulse = 2.0 * Math.PI * kDrumRadius.abs(Units.Meters) / 4096;
  private static final double kCarriageMass = 1.0; // kg
  private static final double kMinHeightMeters = -1.0; // arbitrary: should be < min desired
  private static final double kMaxHeightMeters = 8; // arbitrary: should be > max desired
  private static final boolean ENABLE_GRAVITY = true;
  private static final double kHeightMetersAtStart = 0;

  // Simulation classes help us simulate what's going on, optionally including
  // gravity.
  private final ElevatorSim m_sim = new ElevatorSim(m_gearing, kGearing, kCarriageMass, kDrumRadius.in(Units.Meters),
      kMinHeightMeters, kMaxHeightMeters, ENABLE_GRAVITY, kHeightMetersAtStart);

  /** Creates a new SimulatedElevator. */
  public SimulatedElevator() {

    // Configure the motor.
    var config = new SparkMaxConfig();
    config.closedLoop
        .p(6)
        .i(0)
        .d(0);

    config.encoder.positionConversionFactor(kEncoderMetersPerPulse);
    config.encoder.velocityConversionFactor(kEncoderMetersPerPulse / 60);

    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Simulation rendering setup.
    Mechanism2d rootMech2d = new Mechanism2d(9, 10);
    m_mech2d = rootMech2d.getRoot("LeftClimber Root", 3, 0)
        .append(new MechanismLigament2d("LeftClimber", m_sim.getPositionMeters(), 90));

    // Publish Mechanism2d to SmartDashboard.
    // To show the visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", rootMech2d);
  }

  public void periodic() {
    super.periodic();

    // Update the visualization of the climber positions.
    //
    // This might be done in simulationPeriodic(), since this class *is* purely
    // simulation-oriented. But I'll do it in the periodic() function, as a reminder
    // that this same thing could be done to provide a rendering of the data for a
    // *real* elevator within the SmartDashboard at a match (e.g., as an aid to the
    // drive team).
    m_mech2d.setLength(m_encoder.getPosition());
  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // In this method, we update our simulation of what our subsystem is doing.

    // First we set out "inputs" (voltages).
    final double initialPosition = m_sim.getPositionMeters();
    var appliedOutput = m_motorSim.getAppliedOutput();
    var voltsIn = RoboRioSim.getVInVoltage();
    m_sim.setInput(0, appliedOutput * voltsIn);

    // Next, we update the simulation. The standard loop time is 20ms.
    final double timeIncrement = 0.020;
    m_sim.update(timeIncrement);

    // Per original example, if we don't do this, the rendered angle is off a
    // little bit.
    m_motorSim.setPosition(m_sim.getPositionMeters());

    var motorSpeed = m_sim.getVelocityMetersPerSecond();
    m_motorSim.iterate(motorSpeed, voltsIn, timeIncrement);

    RoboRioSim.setVInVoltage(
        // Note: this should really be updated in conjunction with the simulated drive
        // base (and anything else we're "powering", such as a simulated shooter, as
        // well).
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps()));
  }

  @Override
  protected void resetEncoder_impl() {
    m_encoder.setPosition(0);
  }

  @Override
  protected double getRevolutions_impl() {
    return m_encoder.getPosition();
  }

  @Override
  protected void stop_impl() {
    m_motor.set(0);
  }

  @Override
  protected void extend_impl() {
    m_motor.set(EXTENSION_SPEED);
  }

  @Override
  protected void retract_impl() {
    m_motor.set(RETRACTION_SPEED);
  }

  @Override
  public void setTargetPosition(TargetPosition targetPosition) {

  }
}
