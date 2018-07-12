package org.usfirst.frc.team1072.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * 
 * @author Finn Frankis
 * @version Jul 10, 2018
 */
public class DelayCommand extends CommandGroup
{
    private double delay;
    private Command command;
    
    public DelayCommand (Command command)
    {
        delay = 0;
        this.command = command;
    }
    /**
     * 
     * Constructs a new DelayCommand.
     * @param delay the delay before starting the command, in seconds
     * @param command
     */
    public DelayCommand (double delay, Command command)
    {
        this.delay = delay;
        this.command = command;
    }
    
    public void initialize()
    {
        addSequential(new WaitCommand(delay));
        addSequential(command);
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
