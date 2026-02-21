// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConditionalConstants;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class TransitionRoller extends SubsystemBase {
  SparkMax m_transition;
  public DigitalInput input = new DigitalInput(2);

  /** Creates a new TransitionRoller. */
  public TransitionRoller() {
    if (!ConditionalConstants.SALLY) {
      m_transition =
          new SparkMax(SparkMaxIds.TRANSITION_MOTOR_ID, MotorType.kBrushless);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Beam break", input.get() ? "True" : "False");
  }

  public void setTransitionRollerSpeed(double percentSpeed) {
    System.out.println("Setting speed to " + percentSpeed);
    m_transition.set(percentSpeed);
  }

  public void stop() {
    m_transition.set(0);
  }
}
