// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.CanBusIds.SparkMaxIds;

public class RealDrivebase extends AbstractDrivebase {
  // TODO: add thriftynova support. (This might be done in a derived class, or be
  // based on some information about the robot's configuration. I'd recommend the
  // former approach.)
  //
  // FINDME(Robert): Do you actually need these to be member variables? (Hint: you
  // don't, though you *may* want to have access to the followers temporarily in
  // the constructor, during setup/configuration. But if you don't need them after
  // that, then don't keep them as members of the class; just use local
  // variables.)
  private final SparkMax m_leftfollower;
  private final SparkMax m_rightfollower;

  // TODO: Change these to use the encoders that are associated with the real
  // hardware (i.e., either the relative encoders that are built into the Spark
  // Max hardware, or else the functions that are built into the ThriftyNova motor
  // controller class).
  //
  // Note that Mr. Healy has updated the "TrivialEncoder" class (and some derived
  // classes) so that it can be used with Thrifty Novas (new code this year), as
  // well as the Spark Max controllers, etc. (This stuff is in the sample code
  // under "Experimental/2026/MattsToys/SimulationSupport".)
  private final Encoder m_leftEncoder = new Encoder(1, 2);
  private final Encoder m_rightEncoder = new Encoder(3, 4);
  private final TrivialEncoder m_mainLeftEncoder = TrivialEncoder.forWpiLibEncoder(m_leftEncoder);
  private final TrivialEncoder m_mainRightEncoder = TrivialEncoder.forWpiLibEncoder(m_rightEncoder);

  private final IGyro m_mainGyro;

  @Override
  protected final IGyro getGyro() {
    return m_mainGyro;
  }

  @Override
  protected final TrivialEncoder getLeftEncoder() {
    return m_mainLeftEncoder;
  }

  @Override
  protected final TrivialEncoder getRightEncoder() {
    return m_mainRightEncoder;
  }

  /** Creates a new RealDrivebase. */
  public RealDrivebase() {
    super(new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless),
        new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless));
    // TODO: find actual SparkMax IDs, currents are placeholders.
    //
    // FINDME(Robert): the SparkMax CAN IDs have been held stable for the last few
    // years, in order to let us continue using Sally as a test bed that is
    // compatible with a new year's robot. As a result, you should be able to copy
    // them over from named constants defined in last year's code. (Look in
    // Constants.java for the "Constants.CanBusIds.SparkMaxIds" definitions.)
    m_leftfollower = new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    m_rightfollower = new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
    m_leftEncoder.setDistancePerPulse(getDistancePerPulse());
    m_rightEncoder.setDistancePerPulse(getDistancePerPulse());

    AnalogGyro gyro = new AnalogGyro(0);
    m_mainGyro = IGyro.wrapGyro(gyro);

    // TODO: Configure the motor controllers on the left/right sides (e.g., ensuring
    // that "leader/follower" is set up in case a controller gets swapped out,
    // making sure that "inverted" is set correctly for each side, etc.).

    // TODO(DISCUSS): What about our encoders are missing information here...
  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
