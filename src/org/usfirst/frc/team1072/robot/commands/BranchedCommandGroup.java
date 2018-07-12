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
    private ArrayList<ArrayList<Command>> branches;
    private ArrayList<Integer> currentCommandIndices;
    private ArrayList<Boolean> hasCommandStarted;
    private ArrayList<Boolean> wasCommandRunning;
    
    public BranchedCommandGroup()
    {
        branches = new ArrayList<ArrayList<Command>>();
        currentCommandIndices = new ArrayList<Integer>();
        hasCommandStarted = new ArrayList<Boolean>();
        wasCommandRunning = new ArrayList<Boolean>();
        
    }
    public void addBranch(ArrayList<Command> branch)
    {
        branches.add(branch);
        currentCommandIndices.add(0);
        hasCommandStarted.add(false);
        wasCommandRunning.add(false);
    }
    
    public void execute()
    {
        for (int i = 0; i < branches.size(); i++)
        {
            ArrayList<Command> currentBranch = branches.get(i);
            double currentIndex = currentCommandIndices.get(i);
            if (currentIndex != -1 && currentIndex < currentBranch.size())
            {
                Command currentCommand = currentBranch.get(currentCommandIndices.get(i));
                if (!hasCommandStarted.get(i))
                {
                    hasCommandStarted.set(i, true);
                    currentCommand.start();
                }
                if (currentCommand.isRunning())
                    wasCommandRunning.set(i, true);
                if (!currentCommand.isRunning() && wasCommandRunning.get(i))
                {
                    currentCommandIndices.set(i, currentCommandIndices.get(i)+1);
                    hasCommandStarted.set(i, false);
                    wasCommandRunning.set(i,  false);
                }

            }
            else
                currentCommandIndices.size();
        }
    }
    
    
    /**
    * @return
    */
    @Override
    public boolean isFinished()
    {
        boolean isFinished = true;
        for (int i : currentCommandIndices)
        {
            isFinished = isFinished && (i == -1);
        }
        return isFinished;
    }

}
