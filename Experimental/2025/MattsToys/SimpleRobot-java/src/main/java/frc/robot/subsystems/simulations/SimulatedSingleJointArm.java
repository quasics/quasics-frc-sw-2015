// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.utils.RevSupportFunctions.configureForRadians;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ISingleJointArm;
import frc.robot.subsystems.simulations.SimulationUxSupport.DeviceStatus;
import frc.robot.utils.RobotConfigs.PIDConfig;
import frc.robot.utils.RobotConfigs.RobotConfig;

/**
 * Subsystem simulating a single-joint arm.
 *
 * Note: this is based on example at
 * https://github.com/aesatchien/FRC2429_2025/tree/main/test_robots/sparksim_test.
 */
public class SimulatedSingleJointArm extends SubsystemBase implements ISingleJointArm {

  /** Motor controller running the arm. */
  protected final SparkMax m_motorController = new SparkMax(SimulationPorts.ARM_CAN_ID, MotorType.kBrushless);

  ////////////////////////////////////////////////////////////////////////////////////
  // Values defining the arm's characteristics/physics
  ////////////////////////////////////////////////////////////////////////////////////

  /**
   * Angle reported by the arm when it is fully extended from the robot's frame.
   */
  final Angle ARM_OUT_ANGLE = Degrees.of(180);

  /**
   * Angle reported by the arm when it is fully upright and within the robot's
   * frame.
   */
  final Angle ARM_UP_ANGLE = Degrees.of(90);

  /** Gearing used to drive the arm's motion. */
  final static double GEARING = 5 * 5 * 3 * 4.44; // Arbitrary (but needs to be enough for simulated physics to work)

  /**
   * Length of the arm (used for simulation, but defined here because we'd want to
   * simulate the actual hardware's physics).
   */
  final double ARM_LENGTH_METERS = 1.0; // Currently arbitrary

  /**
   * Mass of the arm (used for simulation, but defined here because we'd want to
   * simulate the actual hardware's physics).
   */
  final double ARM_MASS_KG = 4.0; // Currently arbitrary

  final static Angle MIN_ANGLE = Degrees.of(80);
  final static Angle MAX_ANGLE = Degrees.of(190);
  final static Angle ARM_RANGE = MAX_ANGLE.minus(MIN_ANGLE);

  final static Angle STARTING_ANGLE = MIN_ANGLE.plus(ARM_RANGE.div(2));

  final boolean SIMULATE_GRAVITY = true;

  /** Reference/target position for arm. (Saved for logging purposes.) */
  private Angle m_referencePosition = Degrees.of(0);

  ////////////////////////////////////////////////////////////////////////////////////
  // Simulation support objects
  ////////////////////////////////////////////////////////////////////////////////////

  /** Motor being driven by the controller. */
  private DCMotor m_armPlant = DCMotor.getNEO(1);

  /** Simulation engine for the arm. */
  private SingleJointedArmSim m_armSim = new SingleJointedArmSim(m_armPlant, GEARING,
      SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG), ARM_LENGTH_METERS,
      MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians), SIMULATE_GRAVITY, STARTING_ANGLE.in(Radians));

  /** Simulation driver for the motor controller. */
  private SparkMaxSim m_sparkSim = new SparkMaxSim(m_motorController, m_armPlant);

  /**
   * Creates a new SimulatedSingleJointArm.
   * 
   * @param robotConfig the configuration for the robot being targeted
   */
  public SimulatedSingleJointArm(RobotConfig robotConfig) {
    setName(SUBSYSTEM_NAME);

    final PIDConfig pidConfig = robotConfig.arm().pid();

    // Configure the motor.
    var motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop
        .p(pidConfig.kP())
        .i(pidConfig.kI())
        .d(pidConfig.kD());

    configureForRadians(motorConfig);

    m_motorController.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // By default, hold the arm in our starting position
    setTargetPosition(STARTING_ANGLE);

    //
    // Configure simulation support
    //
    configureSimulation();
  }

  /** Configures simulation objects. */
  private void configureSimulation() {
    m_sparkSim.setPosition(STARTING_ANGLE.in(Radians));
    m_sparkSim.enable();
  }

  /** Determines if debugging output is produced under simulation. */
  final static boolean NOISY = false;

  private void updateSimulation() {
    // Compute the changes in this iteration for the simulated hardware, based on
    // current state.
    final Angle preAngle = Radians.of(m_armSim.getAngleRads());
    var appliedOutput = m_sparkSim.getAppliedOutput();
    var voltsIn = RoboRioSim.getVInVoltage();
    m_armSim.setInput(0, appliedOutput * voltsIn);
    final double timeIncrement = 0.020;
    m_armSim.update(timeIncrement);
    final Angle postAngle = Radians.of(m_armSim.getAngleRads());

    // Apply the computed changes to the hardware simulation.
    var armVelocity = m_armSim.getVelocityRadPerSec();
    m_sparkSim.iterate(armVelocity, voltsIn, timeIncrement);

    // As indicated in the original example, if we don't do this, the rendered angle
    // is off a little bit. (Unclear *why* this should happen, but it definitely
    // *does*.)
    m_sparkSim.setPosition(postAngle.in(Radians));

    // Update the rendering.
    boolean closeEnough = postAngle.isNear(m_referencePosition, Degrees.of(0.5));
    SimulationUxSupport.instance.updateArm(postAngle,
        closeEnough ? DeviceStatus.AtSetpoint : DeviceStatus.NotAtSetpoint);

    // // Note: this should actually be calculated across *all* of the draws, which
    // // implies some sort of centralized physics would be useful.
    final double currentDraw = m_armSim.getCurrentDrawAmps();
    SimulationUxSupport.instance.postCurrentDraw(currentDraw);

    if (NOISY) {
      System.out.println("Target: " + m_referencePosition.in(Degrees) + ", vel: " + armVelocity
          + ", pre angle: " + preAngle.in(Degrees) + ", post angle: " + postAngle.in(Degrees)
          + ", current: " + currentDraw);
    }
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    if (!DriverStation.isDisabled()) {
      updateSimulation();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // Methods from ISingleJointArm
  ////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void setTargetPosition(Angle targetPosition) {
    m_referencePosition = targetPosition;
    m_motorController.getClosedLoopController().setReference(
        targetPosition.in(Radians), SparkBase.ControlType.kPosition);
  }

  @Override
  public Angle getArmOutAngle() {
    return ARM_OUT_ANGLE;
  }

  @Override
  public Angle getArmUpAngle() {
    return ARM_UP_ANGLE;
  }
}
