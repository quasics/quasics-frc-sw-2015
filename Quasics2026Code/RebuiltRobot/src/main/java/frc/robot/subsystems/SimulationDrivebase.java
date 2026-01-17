// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulationDrivebase extends AbstractDrivebase {
  private Encoder m_leftEncoder = new Encoder(1, 2);
  private Encoder m_rightEncoder = new Encoder(3, 4);
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
      KitbotGearing.k12p75, // 12.75:1 if this changes, we may have to use a new diffDrivetrain sim
      KitbotWheelSize.kSixInch, // 6" diameter wheels.
      null // No measurement noise.
  );

  private DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26));

  public void arcadeDrive(LinearVelocity forwardspeed, AngularVelocity turnspeed) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics
        .toWheelSpeeds(new ChassisSpeeds(forwardspeed, LinearVelocity.ofBaseUnits(0.0, null), turnspeed));

    // TODO: Give the motors (it is okay that they are not real, their set/get
    // functions still work) the wheel speeds
  }

  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    // TODO: Read tutorial
    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/updating-drivetrain-model.html

    // TODO: inputs to driveSim
    // TODO: update driveSim

  }

}
