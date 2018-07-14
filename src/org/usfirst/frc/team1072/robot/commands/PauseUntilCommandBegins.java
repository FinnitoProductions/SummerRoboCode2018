package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.commands.PauseUntilPathBeginsCommand.PauseType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 13, 2018
 */
public class PauseUntilCommandBegins extends Command
{
    private Command c;
    private double delay;
    private double startTime;
    private boolean hasInitializedStartTime;
    
    public PauseUntilCommandBegins(Command c, double delay)
    {
        this.c = c;
        this.delay = delay;
    }
    
    public void initialize()
    {
        hasInitializedStartTime = false;
        startTime = Double.MAX_VALUE;
    }
    
    public void execute ()
    {
        if (!hasInitializedStartTime && c.isRunning())
        {
            startTime = Timer.getFPGATimestamp();
            hasInitializedStartTime = true;
        }
    }
    /**
    * @return
    */
    @Override
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return hasInitializedStartTime && 
                !c.isRunning() && 
                Timer.getFPGATimestamp() - startTime >= delay;
    }
    
}
