// Copyright (c) 2026, Quasics Robotics and other contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.real;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.CanBusIds.SparkMaxIds;
import frc.robot.sensors.IGyro;
import frc.robot.sensors.TrivialEncoder;

public class SparkMaxDrivebase extends AbstractDrivebase {
  // FINDME(Robert): Do you actually need these to be member variables? (Hint:
  // you don't, though you *may* want to have access to the followers
  // temporarily in the constructor, during setup/configuration. But if you
  // don't need them after that, then don't keep them as members of the class;
  // just use local variables.)
  private final SparkMax m_leftfollower;
  private final SparkMax m_rightfollower;

  // TODO: Change these to use the encoders that are associated with the real
  // hardware (i.e., either the relative encoders that are built into the Spark
  // Max hardware, or else the functions that are built into the ThriftyNova
  // motor controller class).
  //
  // Note that Mr. Healy has updated the "TrivialEncoder" class (and some
  // derived classes) so that it can be used with Thrifty Novas (new code this
  // year), as well as the Spark Max controllers, etc. (This stuff is in the
  // sample code under "Experimental/2026/MattsToys/SimulationSupport".)
  private final TrivialEncoder m_leftEncoder;
  private final TrivialEncoder m_rightEncoder;

  private final IGyro m_gryo;

  @Override
  protected final IGyro getGyro() {
    return m_gryo;
  }

  @Override
  protected final TrivialEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  @Override
  protected final TrivialEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /** Creates a new SparkMaxDrivebase. */
  public SparkMaxDrivebase() {
    super(new SparkMax(SparkMaxIds.LEFT_LEADER_ID, MotorType.kBrushless),
        new SparkMax(SparkMaxIds.RIGHT_LEADER_ID, MotorType.kBrushless));
    m_leftfollower = new SparkMax(SparkMaxIds.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    m_rightfollower = new SparkMax(SparkMaxIds.RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    Encoder leftEncoder = new Encoder(1, 2);
    Encoder rightEncoder  = new Encoder(3, 4);

    leftEncoder.setDistancePerPulse(getDistancePerPulse());
    rightEncoder.setDistancePerPulse(getDistancePerPulse());

    m_leftEncoder = TrivialEncoder.forWpiLibEncoder(leftEncoder);
    m_rightEncoder = TrivialEncoder.forWpiLibEncoder(rightEncoder);

    AnalogGyro gyro = new AnalogGyro(0);
    m_gryo = IGyro.wrapGyro(gyro);

    // TODO: Configure the motor controllers on the left/right sides (e.g.,
    // ensuring that "leader/follower" is set up in case a controller gets
    // swapped out, making sure that "inverted" is set correctly for each side,
    // etc.).

    // TODO(DISCUSS): What about our encoders are missing information here...
  }

  // We've removed @Override periodic, but be sure to use super.periodic if we
  // ever need to resurrect it.
}
