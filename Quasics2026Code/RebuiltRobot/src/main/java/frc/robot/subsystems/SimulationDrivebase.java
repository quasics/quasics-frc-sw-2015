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
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulationDrivebase extends AbstractDrivebase {
  private PWMSparkMax m_left = new PWMSparkMax(1);
  private PWMSparkMax m_right = new PWMSparkMax(2);


  private Encoder m_leftEncoder = new Encoder(1, 2);
  private Encoder m_rightEncoder = new Encoder(3, 4);
  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
  private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  private AnalogGyroSim m_GyroSim = new AnalogGyroSim(0);

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
        double leftPercent;
        double rightPercent;
        leftPercent = mpsToPercent(wheelSpeeds.leftMetersPerSecond);
        rightPercent = mpsToPercent(wheelSpeeds.rightMetersPerSecond);
        m_leftMotor.set(leftPercent);
        m_rightMotor.set(rightPercent);
  }

  /** Creates a new SimulationDrivebase. */
  public SimulationDrivebase() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(), m_rightMotor.get() * RobotController.getInputVoltage());
    m_driveSim.update(0.02);
    //getHeading returns counterclockwise positive, Gyros are clockwise positive
    m_GyroSim.setAngle(m_driveSim.getHeading().getDegrees());
    //m_leftEncoderSim.setDistance(m_driveSim().getLeftPosition).setRate();
    // TODO: Read tutorial
    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/updating-drivetrain-model.html

    // TODO: inputs to driveSim
    // TODO: update driveSim

  }

}
