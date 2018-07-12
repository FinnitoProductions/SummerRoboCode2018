package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a branch of sequential commands, where multiple branches can be run in parallel.
 * @author Finn Frankis
 * @version Jul 11, 2018
 */
public class Branch
{
    private ArrayList<Command> branch;
    private int currentIndex; // 
    private boolean hasStartedCommand; // whether the command at currentIndex has started
    private boolean didCommandRun; // whether the command at currentIndex has run at any point
    
    public Branch()
    {
        currentIndex = 0;
        hasStartedCommand = false;
        didCommandRun = false;
    }
    
    public Branch addCommand(Command c)
    {
        branch.add(c);
        return this;
    }
    
    public int getCurrentIndex()
    {
        return currentIndex;
    }

    public void incrementCurrentIndex()
    {
        currentIndex++;
    }
    
    public void setCurrentIndex (int value)
    {
        currentIndex = value;
    }

    public boolean isHasStartedNextCommand()
    {
        return hasStartedCommand;
    }

    public void setHasStartedNextCommand(boolean hasStartedNextCommand)
    {
        this.hasStartedCommand = hasStartedNextCommand;
    }

    public boolean isDidCommandRun()
    {
        return didCommandRun;
    }

    public void setCommandRun(boolean didCommandRun)
    {
        this.didCommandRun = didCommandRun;
    }

    
}
