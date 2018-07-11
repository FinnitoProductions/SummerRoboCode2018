package org.usfirst.frc.team1072.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 10, 2018
 */
public class DelayCommand extends Command
{
    private double delay;
    private Command command;
    
    public DelayCommand (Command command)
    {
        delay = 0;
        this.command = command;
    }
    public DelayCommand (double delay, Command command)
    {
        this.delay = delay;
        this.command = command;
    }
    
    public void initialize()
    {
        try
        {
            Thread.sleep((long)delay);
        }
        catch (InterruptedException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        command.start();
    }
    /**
    * @return
    */
    @Override
    protected boolean isFinished()
    {
        return command.isCompleted();
    }
    
    @Override
    protected void end ()
    {
        command.cancel();
    }
}
