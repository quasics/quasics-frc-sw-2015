// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Lighting;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses the lighting subsystem. */
public class SimpleLighting extends CommandBase {
    private final Lighting m_subsystem;
    private final Mode mode;

    public enum Mode {
        Green(0, 255, 0),
        Red(255, 0, 0),
        Blue(0, 0, 255),
        White(255, 255, 255),
        Black(0, 0, 0);

        final private int r, g, b;

        Mode(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public int getR() {
            return r;
        }

        public int getG() {
            return g;
        }

        public int getB() {
            return b;
        }
    }

    /**
     * Creates a new SimpleLighting.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SimpleLighting(Lighting subsystem) {
        this(subsystem, Mode.Green);
    }

    /**
     * Creates a new SimpleLighting.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SimpleLighting(Lighting subsystem, Mode mode) {
        m_subsystem = subsystem;
        this.mode = mode;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.SetStripColor(mode.getR(), mode.getG(), mode.getB());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // TODO: Evaluate if we can instead return true. (I think
        // that should be possible, since once the color is set, I
        // would *hope* that it stays set.)
        return false;
    }
}
