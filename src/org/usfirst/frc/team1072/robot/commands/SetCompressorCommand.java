package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Represents a command to enable and disable the compressor.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class SetCompressorCommand extends InstantCommand
{
    boolean state;
    
    /**
     * Initializes the command, requiring the pneumatics subsystem.
     */
    public SetCompressorCommand(boolean state) 
    { 
        this.state = state;
        requires(Intake.pn); 
    }
    
    /**
     * Executes the command given the compressor's state.
     * @param state the state: true if running; false if not running
     */
    public void execute() 
    { 
        Intake.pn.setCompressor(state);
    }
}
