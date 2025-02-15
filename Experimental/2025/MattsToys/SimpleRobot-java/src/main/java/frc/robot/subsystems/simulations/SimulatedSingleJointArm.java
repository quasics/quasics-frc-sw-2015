// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.ISingleJointArm;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// Based on example at https://github.com/aesatchien/FRC2429_2025/tree/main/test_robots/sparksim_test.
public class SimulatedSingleJointArm extends SubsystemBase implements ISingleJointArm {
  final double MAX_ANGLE_RADIANS = Math.toRadians(80);
  final double MIN_ANGLE_RADIANS = Math.toRadians(190);
  final double STARTING_ANGLE_RADIANS = (MIN_ANGLE_RADIANS - MAX_ANGLE_RADIANS) / 2 + MAX_ANGLE_RADIANS;
  final boolean SIMULATE_GRAVITY = true;

  /** Motor controller running the arm. */
  private SparkMax m_motorController = new SparkMax(0, MotorType.kBrushless);

  /** Reference/target position for arm. (Saved for logging purposes.) */
  private double m_referencePosition = 0;

  ////////////////////////////////////////////////////////////////////////////////////
  // Simulation support objects
  ////////////////////////////////////////////////////////////////////////////////////

  /** Motor being driven by the controller. */
  private DCMotor armPlant = DCMotor.getNEO(1);

  /** Simulation engine for the arm. */
  private SingleJointedArmSim armSim = new SingleJointedArmSim(
      armPlant, GEARING,
      SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG), ARM_LENGTH_METERS, MAX_ANGLE_RADIANS,
      MIN_ANGLE_RADIANS, SIMULATE_GRAVITY, STARTING_ANGLE_RADIANS);

  /** Simulation driver for the motor controller. */
  private SparkMaxSim sparkSim = new SparkMaxSim(m_motorController, armPlant);

  /** Smart Dashboard UI component showing the arm's position. */
  private final MechanismLigament2d crankMech2d;

  /** Creates a new SimulatedSingleJointArm. */
  public SimulatedSingleJointArm() {
    // Configure the motor.
    var config = new SparkMaxConfig();
    config.closedLoop
        .p(6)
        .i(0)
        .d(0);

    // Note: the SparkSim derives the gear ratio based on the ratio between
    // positionconversionfactor and velocityconversionfactor. As a result,
    // we need to make sure that these are set in order for the simulation
    // to work correctly.
    config.encoder.positionConversionFactor(2 * Math.PI / GEARING);
    config.encoder.velocityConversionFactor(2 * Math.PI / (60 * GEARING));

    m_motorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // By default, hold the arm in our starting position
    setTargetPositionInRadians(STARTING_ANGLE_RADIANS);

    //
    // Configure simulation support
    //
    configureSimulation();
    crankMech2d = configureSmartDashboardWidgets();
  }

  /** Configures simulation objects. */
  private void configureSimulation() {
    sparkSim.setPosition(STARTING_ANGLE_RADIANS);
    sparkSim.enable();
  }

  /** @#return the ligament for the arm on the smart dashboard. */
  private MechanismLigament2d configureSmartDashboardWidgets() {
    Mechanism2d armMech2d = new Mechanism2d(60, 60);
    MechanismRoot2d root = armMech2d.getRoot("root", 40, 10);
    var baseMech2d = root.append(new MechanismLigament2d("frame", -20, 0));
    var crankMech2d = baseMech2d.append(new MechanismLigament2d("crank", 20, armSim.getAngleRads()));
    baseMech2d.setColor(new Color8Bit(200, 200, 200));
    SmartDashboard.putData("Arm", armMech2d);

    return crankMech2d;
  }

  /**
   * Sets the target position (angle) for the arm.
   * 
   * @param targetPosition target arm position (in radians)
   */
  @Override
  public void setTargetPositionInRadians(double targetPosition) {
    m_referencePosition = targetPosition;
    m_motorController.getClosedLoopController().setReference(targetPosition, SparkBase.ControlType.kPosition);
  }

  /** Determines if debugging output is produced under simulation. */
  final static boolean NOISY = false;

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    // Compute the changes to the simulated hardware.
    final double preAngle = armSim.getAngleRads();
    var appliedOutput = sparkSim.getAppliedOutput();
    var voltsIn = RoboRioSim.getVInVoltage();
    armSim.setInput(0, appliedOutput * voltsIn);
    final double timeIncrement = 0.020;
    armSim.update(timeIncrement);

    // Per original example, if we don't do this, the rendered angle is off a
    // little bit.
    sparkSim.setPosition(armSim.getAngleRads());

    var armVelocity = armSim.getVelocityRadPerSec();
    sparkSim.iterate(armVelocity, voltsIn, timeIncrement);

    // // Note: this should actually be calculated across *all* of the draws, which
    // // implies some sort of centralized physics would be useful.
    // final double currentDraw = armSim.getCurrentDrawAmps();
    // RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
    // // List of current draws (in amps)
    // currentDraw));

    final double postAngle = armSim.getAngleRads();
    crankMech2d.setAngle(Math.toDegrees(postAngle));

    if (NOISY) {
      System.out.println("Target: " + m_referencePosition +
          ", vel: " + armVelocity +
          ", pre angle: " + preAngle +
          ", post angle: " + postAngle);
    }
  }
}
