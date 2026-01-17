// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.SingleMotor;

public class RobotContainer {
  SingleMotor m_singleMotor = new SingleMotor();

  public RobotContainer() {
    addPowerButton("Stop!", 0);
    addPowerButton("-25% power", -.25);
    addPowerButton("-50% power", -.5);
    addPowerButton("-100% power", -1.0);
    addPowerButton("+25% power", +.25);
    addPowerButton("+50% power", +.5);
    addPowerButton("+100% power", +1.0);
  }

  private void addPowerButton(String label, double percent) {
    SmartDashboard.putData(label,
        new FunctionalCommand(
            // onInit (can't be null)
            ()
                -> { m_singleMotor.setSpeed(percent); },
            // onExecute (can't be null)
            ()
                -> { m_singleMotor.setSpeed(percent); },
            // onEnd (can't be null)
            (Boolean b)
                -> { m_singleMotor.stop(); },
            // isFinished (can't be null)
            ()
                -> false,
            // Dependency
            m_singleMotor));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
