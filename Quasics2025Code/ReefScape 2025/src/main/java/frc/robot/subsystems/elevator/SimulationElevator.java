// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

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

public class SimulationElevator extends AbstractElevator {
  static final double EXTENSION_SPEED = +1.0;
  static final double RETRACTION_SPEED = -1.0;
  static final double MAX_DESIRED_HEIGHT = 2.0;
  static final double MIN_DESIRED_HEIGHT = 0.0;
  static final double ACCEPTABLE_ERROR = 0.1;

  static public final int ELEVATOR_PWM_ID = 5;
  static public final int ELEVATOR_ENCODER_PORT_A = 4;
  static public final int ELEVATOR_ENCODER_PORT_B = 5;

  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_gearing = DCMotor.getNEO(1);

  private final Encoder m_encoder = new Encoder(ELEVATOR_ENCODER_PORT_A, ELEVATOR_ENCODER_PORT_B);
  private final PWMSparkMax m_motor = new PWMSparkMax(ELEVATOR_PWM_ID);

  // Mechanism2d visualization of the hardware (for rendering in
  // SmartDashboard, or the simulator).
  private final MechanismLigament2d m_mech2d;

  //////////////////////////////////////////////////////////////////////////////
  // Simulation support data/objects

  // Simulation motors/encoders
  private final PWMSim m_motorSim = new PWMSim(m_motor);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // TODO: Update these constants to better emulate the real behavior of the
  // hardware. (But for now, this will at least give us something we can use.)
  private static final double kGearing = 10.0;
  private static final Distance kDrumRadius = Units.Inches.of(1);
  private static final double kEncoderMetersPerPulse = 2.0 * Math.PI * kDrumRadius.abs(Units.Meters) / 4096;
  private static final double kCarriageMass = 1.0; // kg
  private static final double kMinHeightMeters = MIN_DESIRED_HEIGHT - 1; // arbitrary: should be < min desired
  private static final double kMaxHeightMeters = MAX_DESIRED_HEIGHT + 1; // arbitrary: should be > max desired
  private static final boolean ENABLE_GRAVITY = true;
  private static final double kHeightMetersAtStart = 0;

  // Simulation classes help us simulate what's going on, optionally including
  // gravity.
  private final ElevatorSim m_sim = new ElevatorSim(m_gearing, kGearing, kCarriageMass, kDrumRadius.in(Units.Meters),
      kMinHeightMeters, kMaxHeightMeters, ENABLE_GRAVITY, kHeightMetersAtStart);

  private TargetPosition m_targetPosition = TargetPosition.kDontCare;

  /** Creates a new SimulationElevator. */
  public SimulationElevator() {
    // Encoder setup (so that simulation can drive actual values; without this,
    // we'll keep getting 0 distance/height, regardless of
    // direction/speed/duration).
    m_encoder.setDistancePerPulse(kEncoderMetersPerPulse);

    // Simulation rendering setup.
    Mechanism2d rootMech2d = new Mechanism2d(9, 10);
    m_mech2d = rootMech2d.getRoot("LeftClimber Root", 3, 0)
        .append(new MechanismLigament2d("LeftClimber", m_sim.getPositionMeters(), 90));

    // Publish Mechanism2d to SmartDashboard.
    // To show the visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", rootMech2d);
  }

  protected double translateTargetPositionToValue(TargetPosition position) {
    switch (position) {
      case kDontCare:
        return m_encoder.getDistance();
      // Actual values start here.
      case kBottom:
        return MIN_DESIRED_HEIGHT;
      case kL1:
        return (MAX_DESIRED_HEIGHT - MIN_DESIRED_HEIGHT) / 2;
      case kL2:
        return MAX_DESIRED_HEIGHT;
      case kTop:
        return MAX_DESIRED_HEIGHT;
    }

    System.err.println("**** Invalid/unexpected target position: " + position);
    return 0;
  }

  private void updateTargetedSpeed() {
    if (m_targetPosition != TargetPosition.kDontCare) {
      final double targetValue = translateTargetPositionToValue(m_targetPosition);
      final double error = targetValue - m_encoder.getDistance();
      final double percentError = error / (MAX_DESIRED_HEIGHT - MIN_DESIRED_HEIGHT); // Complete hack
      final double errorScaling = 1.0;
      double speed = Math.min(Math.max(percentError * errorScaling, -1.0), 1.0);
      if (ACCEPTABLE_ERROR > Math.abs(error)) {
        speed = 0;
      }
      m_motor.set(speed);
    }
  }

  public void periodic() {
    super.periodic();

    // TODO: Emulating PID control for now. Real PID control can be added later.
    if (m_targetPosition != TargetPosition.kDontCare) {
      updateTargetedSpeed();
    }

    // Update the visualization of the climber positions.
    //
    // This might be done in simulationPeriodic(), since this class *is* purely
    // simulation-oriented. But I'll do it in the periodic() function, as a reminder
    // that this same thing could be done to provide a rendering of the data for a
    // *real* elevator within the SmartDashboard at a match (e.g., as an aid to the
    // drive team).
    m_mech2d.setLength(m_encoder.getDistance());
  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // In this method, we update our simulation of what our subsystem is doing.

    // First, we set our "inputs" (voltages).
    final double speed = m_motorSim.getSpeed();
    final double batteryVoltage = RobotController.getBatteryVoltage();
    final double desiredVoltage = speed * batteryVoltage;
    m_sim.setInput(desiredVoltage);

    // Next, we update the simulation. The standard loop time is 20ms.
    m_sim.update(0.020);

    // Now, we can set our simulated encoder's readings and simulated battery
    // voltage.
    final double leftPos = m_sim.getPositionMeters();
    m_encoderSim.setDistance(leftPos);

    RoboRioSim.setVInVoltage(
        // Note: this should really be updated in conjunction with the simulated drive
        // base (and anything else we're "powering", such as a simulated shooter, as
        // well).
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps()));
  }

  @Override
  public void setTargetPosition(TargetPosition position) {
    m_targetPosition = position;
  }

  @Override
  public void setSpeed(double percentSpeed) {
    m_motor.set(percentSpeed);

    // Client code is presumably directly controlling the elevator, so we won't try
    // to use PID to drive to a given set point. (If we don't do this, then the PID
    // logic in "periodic()" is going to immediately override the speed, if a target
    // position has previously been set.)
    m_targetPosition = TargetPosition.kDontCare;
  }

  @Override
  public void stop() {
    m_motor.set(0);
  }

  @Override
  public void resetEncoders() {
    m_encoder.reset();
  }

  @Override
  public double getPosition() {
    return m_encoder.getDistance();
  }

  @Override
  public double getVelocity() {
    return m_encoder.getRate();
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
