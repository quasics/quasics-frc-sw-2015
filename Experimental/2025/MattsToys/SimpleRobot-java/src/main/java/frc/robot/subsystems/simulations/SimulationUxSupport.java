// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.abstracts.AbstractElevator;

/** Add your docs here. */
public class SimulationUxSupport {

  private final List<Double> m_currentDraws = new LinkedList<Double>();

  /**
   * Mechanism2d visualization of the hardware (for rendering in SmartDashboard,
   * or the simulator).
   */
  private final MechanismLigament2d m_elevatorMech2d;
  private final MechanismLigament2d m_armMech2d;

  /** Color used to render the elevator when running under manual control. */
  private final static Color8Bit NO_SETPOINT = new Color8Bit(255, 165, 0);

  /**
   * Color used to render the elevator when we've reached the target position .
   */
  private final static Color8Bit AT_SETPOINT = new Color8Bit(0, 255, 0);

  /**
   * Color used to render the elevator when we're driving towards the target
   * position .
   */
  private final static Color8Bit NOT_AT_SETPOINT = new Color8Bit(255, 0, 0);

  public static final SimulationUxSupport instance = new SimulationUxSupport();

  private SimulationUxSupport() {
    // Simulation rendering setup.
    Mechanism2d rootMech2d = new Mechanism2d(
        9,
        AbstractElevator.MAX_SAFE_HEIGHT.in(Meters) * 1.15 // Leave a little room at the top
    );
    m_elevatorMech2d = rootMech2d.getRoot("Root", 5, 0)
        .append(new MechanismLigament2d("LeftClimber", 0, 90));

    m_armMech2d = m_elevatorMech2d.append(
        new MechanismLigament2d("Arm", 3, -90));

    // Publish Mechanism2d to SmartDashboard.
    // To show the visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", rootMech2d);

    updateElevator(Meters.of(0), DeviceStatus.Manual);
    updateArm(Degrees.of(0), DeviceStatus.Manual);
  }

  public enum DeviceStatus {
    Manual, AtSetpoint, NotAtSetpoint
  }

  final static private double ARM_ANGLE_OFFSET = -90;

  public void updateArm(Angle angle, DeviceStatus status) {
    m_armMech2d.setAngle(angle.in(Degrees) + ARM_ANGLE_OFFSET);
    updateDeviceStatus(m_armMech2d, status);
  }

  public void updateElevator(Distance height, DeviceStatus status) {
    m_elevatorMech2d.setLength(height.in(Meters));
    updateDeviceStatus(m_elevatorMech2d, status);
  }

  private static void updateDeviceStatus(MechanismLigament2d mech, DeviceStatus status) {
    switch (status) {
      case Manual:
        mech.setColor(NO_SETPOINT);
        break;
      case AtSetpoint:
        mech.setColor(AT_SETPOINT);
        break;
      case NotAtSetpoint:
        mech.setColor(NOT_AT_SETPOINT);
        break;
    }
  }

  public void postCurrentDraw(double currentDraw) {
    m_currentDraws.add(currentDraw);
  }

  public List<Double> getCurrentDraws() {
    return Collections.unmodifiableList(m_currentDraws);
  }

  public void clearCurrentDraws() {
    m_currentDraws.clear();
  }

  public void updateBatteryVoltageFromDraws() {
    if (m_currentDraws.size() > 0) {
      double drawsAsArray[] = m_currentDraws.stream().mapToDouble(Double::doubleValue).toArray();
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(drawsAsArray));
      m_currentDraws.clear();
    }
  }
}
