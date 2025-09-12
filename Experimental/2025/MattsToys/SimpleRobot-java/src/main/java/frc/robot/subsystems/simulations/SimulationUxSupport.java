// Copyright (c) 2025, Matthew J. Healy and other Quasics contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simulations;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.live.Lighting.StockColor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.abstracts.AbstractElevator;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

/**
 * Support for the simulation's user interface (UX).
 *
 * In the 2025 season, this class was created to allow the simulation (or Smart
 * Dashboard/Shuffleboard) to render the elevator and attached single-joint arm,
 * as well as the current draw of the robot.
 */
public class SimulationUxSupport {
  /** List of all current draws across the simulation. */
  private final List<Double> m_currentDraws = new LinkedList<Double>();

  /**
   * Mechanism2d visualization of the elevator (for rendering in SmartDashboard,
   * or the simulator).
   */
  private final MechanismLigament2d m_elevatorMech2d;

  /**
   * Mechanism2d visualization of the single-joint arm (for rendering in
   * SmartDashboard, or the simulator).
   */
  private final MechanismLigament2d m_armMech2d;

  /** Color used to render the elevator when running under manual control. */
  private final static Color8Bit NO_SETPOINT = new Color8Bit(StockColor.Orange.toWpiColor());

  /**
   * Color used to render the elevator when we've reached the target position .
   */
  private final static Color8Bit AT_SETPOINT = new Color8Bit(StockColor.Green.toWpiColor());

  /**
   * Color used to render the elevator when we're driving towards the target
   * position .
   */
  private final static Color8Bit NOT_AT_SETPOINT = new Color8Bit(StockColor.Red.toWpiColor());

  /** Singleton instance of this class. */
  public static final SimulationUxSupport instance = new SimulationUxSupport();

  /** Constructor. */
  private SimulationUxSupport() {
    final double armLengthMeters = 1;

    // Simulation rendering setup.
    Mechanism2d rootMech2d = new Mechanism2d(9,
        (AbstractElevator.MAX_SAFE_HEIGHT.in(Meters) + armLengthMeters)
            * 1.15 // Leave a little room at the top
    );
    m_elevatorMech2d =
        rootMech2d.getRoot("Root", 5, 0).append(new MechanismLigament2d("LeftClimber", 0, 90));

    m_armMech2d = m_elevatorMech2d.append(new MechanismLigament2d("Arm", armLengthMeters, -90));

    // Publish Mechanism2d to SmartDashboard.
    // To show the visualization, select Network Tables -> SmartDashboard
    // -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", rootMech2d);

    updateElevator(Meters.of(0), DeviceStatus.Manual);
    updateArm(Degrees.of(0), DeviceStatus.Manual);

    // Elevator bound markers
    var lowerBoundRoot = rootMech2d.getRoot("FloorRoot", 0, 0);
    var lowerBound =
        lowerBoundRoot.append(new MechanismLigament2d("Floor", 10, 0, 3, new Color8Bit(255, 0, 0)));
    var topBoundRoot =
        rootMech2d.getRoot("TopRoot", 0, AbstractElevator.MAX_SAFE_HEIGHT.in(Meters));
    var topBound =
        topBoundRoot.append(new MechanismLigament2d("Top", 10, 0, 3, new Color8Bit(0, 0, 255)));
  }

  /**
   * Enumeration of the status of a device (elevator or arm).
   */
  public enum DeviceStatus {
    /** Device is being controlled manually (i.e., not PID). */
    Manual,
    /** Device is at setpoint/target position (PID control). */
    AtSetpoint,
    /** Device is not at setpoint/target position (PID control). */
    NotAtSetpoint
  }

  /** Offset to apply to the arm angle to make it match the simulation. */
  final static private double ARM_ANGLE_OFFSET = -90;

  /**
   * Updates the arm's angle and status in the UX.
   *
   * @param angle  current angle of the arm
   * @param status current status of the arm
   */
  public void updateArm(Angle angle, DeviceStatus status) {
    m_armMech2d.setAngle(angle.in(Degrees) + ARM_ANGLE_OFFSET);
    updateDeviceStatus(m_armMech2d, status);
  }

  /**
   * Updates the elevator's angle and status in the UX.
   *
   * @param height current height of the elevator
   * @param status current status of the elevator
   */
  public void updateElevator(Distance height, DeviceStatus status) {
    m_elevatorMech2d.setLength(height.in(Meters));
    updateDeviceStatus(m_elevatorMech2d, status);
  }

  /**
   * Updates the color of a device (elevator or arm) in the UX in order to reflect
   * its status.
   *
   * @param mech
   * @param status
   */
  private static void updateDeviceStatus(MechanismLigament2d mech, DeviceStatus status) {
    Color8Bit color = switch (status) {
      case Manual -> NO_SETPOINT;
      case AtSetpoint -> AT_SETPOINT;
      case NotAtSetpoint -> NOT_AT_SETPOINT;
    };
    mech.setColor(color);
  }

  /**
   * Adds a current draw to the known set for this cycle.
   *
   * @param currentDraw the current draw to add
   */
  public void postCurrentDraw(double currentDraw) {
    m_currentDraws.add(currentDraw);
  }

  /**
   * Retrieves list of known current draws.
   *
   * @return the known current draws in this cycle
   */
  public List<Double> getCurrentDraws() {
    return Collections.unmodifiableList(m_currentDraws);
  }

  /**
   * Clears the list of known current draws.
   */
  public void clearCurrentDraws() {
    m_currentDraws.clear();
  }

  /**
   * Updates the battery voltage (under simulation), based on the current draws.
   */
  public void updateBatteryVoltageFromDraws() {
    if (m_currentDraws.size() > 0) {
      double drawsAsArray[] = m_currentDraws.stream().mapToDouble(Double::doubleValue).toArray();
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(drawsAsArray));
      m_currentDraws.clear();
    }
  }
}
