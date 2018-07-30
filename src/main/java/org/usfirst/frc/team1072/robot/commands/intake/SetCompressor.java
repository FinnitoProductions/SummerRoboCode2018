package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Represents a command to enable and disable the compressor.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class SetCompressor extends InstantCommand
{
    /**
     * The intended state for the compressor.
     */
    private boolean state;
    
    /**
     * Initializes the command, requiring the pneumatics subsystem.
     * @param state the compressor state: true if running; false if not running
     */
    public SetCompressor(boolean state) 
    { 
        this.state = state;
        requires(Intake.pn); 
    }
    
    /**
     * Executes the command given the compressor's state.
     */
    public void execute() 
    { 
        Intake.pn.setCompressor(state);
    }
}
