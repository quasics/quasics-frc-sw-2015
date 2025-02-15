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

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

// Based on example at https://github.com/aesatchien/FRC2429_2025/tree/main/test_robots/sparksim_test.
public class SimulatedSingleJointArm extends SubsystemBase {
  public static final double minimumAngleRadians = Math.toRadians(80);
  public static final double maximumAngleRadians = Math.toRadians(190);
  private static final double gearing = 5 * 5 * 3 * 4.44; // Arbitrary
  private static final double armLengthMeters = 1.0; // Arbitrary
  private static final double armMassKg = 4.0; // Arbitrary
  private static final double startingAngleRadians = (maximumAngleRadians - minimumAngleRadians) / 2
      + minimumAngleRadians;
  private static final boolean simulateGravity = true;

  private SparkMax sparkMotor = new SparkMax(0, MotorType.kBrushless);
  private double reference = 0;

  // Simulation support
  private DCMotor armPlant = DCMotor.getNEO(1);
  private SingleJointedArmSim armSim = new SingleJointedArmSim(
      armPlant, gearing,
      SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg), armLengthMeters, minimumAngleRadians,
      maximumAngleRadians, simulateGravity, startingAngleRadians);

  private SparkMaxSim sparkSim = new SparkMaxSim(sparkMotor, armPlant);

  private Mechanism2d armMech2d = new Mechanism2d(60, 60);
  private final MechanismLigament2d crankMech2d;

  /** Creates a new SimulatedSingleJointArm. */
  public SimulatedSingleJointArm() {

    var config = new SparkMaxConfig();
    config.closedLoop.pid(1, 0, 0);
    // the sparksim figures out your gear ratio based on the ratio between
    // positionconversionfactor and velocityconversionfactor
    config.encoder.positionConversionFactor(2 * Math.PI / gearing);
    config.encoder.velocityConversionFactor(2 * Math.PI / (60 * gearing));
    sparkMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setTargetPosition(startingAngleRadians);

    // self.test_spark.configure(self.test_spark_config,
    // SparkMax.ResetMode.kResetSafeParameters,
    // persistMode=SparkMax.PersistMode.kPersistParameters)

    sparkSim.setPosition(startingAngleRadians);
    sparkSim.enable();

    MechanismRoot2d root = armMech2d.getRoot("root", 40, 10);
    var baseMech2d = root.append(new MechanismLigament2d("frame", -20, 0));
    crankMech2d = baseMech2d.append(new MechanismLigament2d("crank", 20, armSim.getAngleRads()));
    baseMech2d.setColor(new Color8Bit(200, 200, 200));
    SmartDashboard.putData("Arm", armMech2d);
  }

  public void setTargetPosition(double targetPosition) {
    reference = targetPosition;
    sparkMotor.getClosedLoopController().setReference(targetPosition, SparkBase.ControlType.kPosition);
  }

  final static boolean NOISY = false;

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    final double preAngle = armSim.getAngleRads();

    var appliedOutput = sparkSim.getAppliedOutput();
    var voltsIn = RoboRioSim.getVInVoltage();
    armSim.setInput(0, appliedOutput * voltsIn);
    final double timeIncrement = 0.020;
    armSim.update(timeIncrement);

    // Per original example: if we don't do this, the angle is off by a little
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

    if (NOISY) {
      System.out.println("Target: " + reference +
          ", vel: " + armVelocity +
          ", pre angle: " + preAngle +
          ", post angle: " + postAngle);
    }
    crankMech2d.setAngle(Math.toDegrees(postAngle));
  }
}
