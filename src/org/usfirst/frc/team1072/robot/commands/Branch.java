package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a branch of sequential commands, where multiple branches can be run in parallel.
 * @author Finn Frankis
 * @version Jul 11, 2018
 */
public class Branch extends Command
{
    private ArrayList<Command> branch;
    private int currentIndex; // 
    private boolean hasStartedCommand; // whether the command at currentIndex has started
    private boolean didCommandRun; // whether the command at currentIndex has run at any point
    
    /**
     * Constructs a new Branch starting at the first command.
     */
    public Branch()
    {
        currentIndex = 0;
        hasStartedCommand = false;
        didCommandRun = false;
    }
    
    public void execute()
    {
        if (currentIndex < branch.size())
        {
            Command currentCommand = getCommand(currentIndex);
            if (!hasStartedCommand)
            {
                hasStartedCommand = true;
                currentCommand.start();
            }
            if (currentCommand.isRunning())
                didCommandRun = true;
            if (!currentCommand.isRunning() && didCommandRun)
            {
                incrementCurrentIndex();
                didCommandRun = false;
                didCommandRun = false;
            }

        }
    }
    /**
     * Adds a new command to this branch. Should be called before execution
     * @param c the command to be added
     * @return this branch
     */
    public Branch addCommand(Command c)
    {
        branch.add(c);
        return this;
    }
    
    /**
     * The index of the current command in the branch (how many commands gotten through).
     * @return the index of the current command
     */
    public int getCurrentIndex()
    {
        return currentIndex;
    }

    /**
     * Increments the current index by one.
     */
    public void incrementCurrentIndex()
    {
        currentIndex++;
    }
    
    /**
     * Sets the current index to be a certain value.
     * @param value the value to replace the current index
     */
    public void setCurrentIndex (int value)
    {
        currentIndex = value;
    }

    /**
     * Determines whether the current command has been told to start (using .start) yet.
     * @return true if the command has been told to start; false otherwise
     */
    public boolean isHasStartedCommand()
    {
        return hasStartedCommand;
    }

    /**
     * Sets whether the command has been started yet.
     * @param hasStartedCommand the value to set hasStarted command to
     */
    public void setHasStartedCommand(boolean hasStartedCommand)
    {
        this.hasStartedCommand = hasStartedCommand;
    }

    /**
     * Determines whether the current command has ever been run.
     * @return true if the command has ever been run
     */
    public boolean isDidCommandRun()
    {
        return didCommandRun;
    }

    /**
     * Sets whether the command has ever been run.
     * @param didCommandRun the value to set didCommandRun to
     */
    public void setCommandRun(boolean didCommandRun)
    {
        this.didCommandRun = didCommandRun;
    }
    
    /**
     * Gets the command in the branch at the given index.
     * @param index the index of the command in the branch to be obtained
     * @return the command at index
     */
    public Command getCommand(int index)
    {
        return branch.get(index);
    }

    /**
    * @return true if all the commands in this branch have been executed.
    */
    @Override
    protected boolean isFinished()
    {
        return currentIndex >= branch.size();
    }
    
    

    
}
