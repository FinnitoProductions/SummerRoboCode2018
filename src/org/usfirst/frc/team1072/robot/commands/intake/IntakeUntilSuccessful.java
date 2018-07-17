package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Intakes continuously until the cube has been successfully intaken.
 * @author Finn Frankis
 * @version Jul 10, 2018
 */
public class IntakeUntilSuccessful extends Command
{
    public IntakeUntilSuccessful()
    {
        
    }
    
    
    /**
    * Determines whether the command has finished.
    * @return true if the current limit has exceeded the specified value; false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        return Robot.intake.getLeftTalon().getOutputCurrent() > 10;
    }
    
}
