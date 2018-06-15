package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a command to enable and disable the compressor.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class SetCompressorCommand extends Command
{

    /**
     * Initializes the command, requiring the pneumatics subsystem.
     */
    public SetCompressorCommand()
    {
        requires(Intake.pn);
    }

    /**
     * Determines whether the command is finished.
     */
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return true;
    }
    
    /**
     * Executes the command given the compressor's state.
     * @param state the state: true if running; false if not running
     */
    public void execute(boolean state)
    {
        Intake.pn.setCompressor(state);
    }

}
