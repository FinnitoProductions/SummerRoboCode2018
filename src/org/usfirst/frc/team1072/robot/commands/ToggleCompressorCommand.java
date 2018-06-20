package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a command to toggle the compressor.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class ToggleCompressorCommand extends Command
{

    /**
     * Sets up the command, requiring the pneumatics subsystem.
     */
    public ToggleCompressorCommand() 
    { 
        requires(Intake.pn); 
    }

    /**
     * Determines whether the commmand has finished.
     */
    protected boolean isFinished() { return Intake.pn.getCompressor().getClosedLoopControl(); }

    /**
     * Executes the command to toggle the compressor.
     */
    public void execute() 
    { 
        Intake.pn.setCompressor(!Intake.pn.getCompressor().getClosedLoopControl()); 
    }

}
