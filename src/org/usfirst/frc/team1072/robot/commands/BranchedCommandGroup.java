package org.usfirst.frc.team1072.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 11, 2018
 */
public class BranchedCommandGroup extends Command
{
    private Map<Integer, LinkedList<Command>> branches = new HashMap<Integer, LinkedList<Command>>();
    private Map<Command, Boolean> startedCommands = new HashMap<Command, Boolean>();
    private Map<LinkedList<Command>, Boolean> startedBranches = new HashMap<LinkedList<Command>, Boolean>();
    
    private Stack<LinkedList<Command>> branchesToRemove = new Stack<LinkedList<Command>>();
    
    @Override
    public void execute()
    {
        System.out.println(branches);
        for (Integer i : branches.keySet()) 
        {
            System.out.println(i);
            LinkedList<Command> currentBranch = branches.get(i);
            if (currentBranch != null)
            {
                Command currentCommand = currentBranch.peek();
                if(!startedBranches.containsKey(currentBranch) || !startedBranches.get(currentBranch))
                {
                    currentCommand.start();
                    startedBranches.put(currentBranch, true);
                }
                if (currentCommand != null)
                {
                    if (currentCommand.isRunning())
                        startedCommands.put(currentCommand, true);
                    if ((currentCommand.isCanceled() || !currentCommand.isRunning()) && getStarted(currentCommand))
                    { 
                        currentBranch.remove();
                        startedBranches.put(currentBranch, false);
                    }
                    else
                        branchesToRemove.add(currentBranch);
                }
            }
        }
        while (!branchesToRemove.isEmpty())
        {
            startedBranches.remove(branchesToRemove.pop());
        }
    }
    
    /**
    * @return
    */
    @Override
    protected boolean isFinished()
    {
        boolean isFinished = !branches.isEmpty();
        for (Integer i : branches.keySet())
        {
            for (Command c : branches.get(i))
            {
                isFinished = isFinished && getFinished(c);
            }
        }
        return isFinished;
    }
    
    public boolean getFinished()
    {
        return isFinished();
    }
    
    protected void end()
    {
        for (Integer i : branches.keySet())
        {
            for (Command c : branches.get(i))
            {
                c.cancel();
            }
        }
    }
    
    public BranchedCommandGroup addBranch(int branch, Command c)
    {
        if (!branches.containsKey(branch) || branches.get(branch) == null)
            branches.put(branch, new LinkedList<Command>());
        branches.get(branch).add(c);
        startedCommands.put(c, false);
        startedBranches.put(branches.get(branch), false);
        return this;
    }
    
    public boolean getStarted(Command c)
    {
        return startedCommands.get(c);
    }
    
    public boolean getFinished(Command c)
    {
        if (startedCommands.containsKey(c))
        {
            return getStarted(c) && (c.isCanceled() || !c.isRunning());     
        }
            
        return false;
    }

}
