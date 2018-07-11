package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 10, 2018
 */
public class IntakeUntilSuccessfulCommand extends Command
{
    public IntakeUntilSuccessfulCommand()
    {
        
    }
    
    
    /**
    * @return 
    */
    @Override
    protected boolean isFinished()
    {
        return Robot.intake.getLeftTalon().getOutputCurrent() > 10;
    }
    
}
