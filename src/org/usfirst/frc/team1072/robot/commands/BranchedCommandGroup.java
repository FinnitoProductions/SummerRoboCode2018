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
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * 
 * @author Finn Frankis
 * @version Jul 11, 2018
 */
public class BranchedCommandGroup extends Command
{
    private ArrayList<Branch> branches;
    
    public BranchedCommandGroup()
    {
        branches = new ArrayList<Branch>();
    }
    
    public void addBranch (Branch b)
    {
        branches.add(b);
    }
    
    public void initialize()
    {
        for (Branch b : branches)
        {
            Scheduler.getInstance().add(b);
        }
    }
    
    public void execute()
    {
        /*for (Branch b : branches)
        {
            if (!b.isFinished())
                b.execute();
        }*/
    }
    
    /**
    * Determines whether this command is complete.
    * @return whether this command group has finished
    */
    @Override
    public boolean isFinished()
    {
        boolean isFinished = true;
        for (Branch b : branches)
        {
            isFinished = isFinished && b.isFinished();
        }
        return isFinished;
    }
    
    
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
        private int number;
        private Subsystem required;
        
        /**
         * Constructs a new Branch starting at the first command.
         */
        public Branch()
        {
            initBranch();
            number = branches.size() - 1;
        }
        
        public Branch(int number)
        {
            initBranch();
            for (Branch b : branches)
            {
                if (b.getNumber() == number)
                {
                    try 
                    {
                        throw new InterruptedException ("This number is already in use.");
                    }
                    catch (Exception e)
                    {
                        e.printStackTrace();
                    }
                    
                }
            }
            this.number = number;
        }
        
        public Branch (Subsystem s)
        {
            initBranch();
            requires(s);
            this.setRequired(s);
        }
        


        private void initBranch()
        {
            branch = new ArrayList<Command>();
            currentIndex = 0;
            hasStartedCommand = false;
            didCommandRun = false;
        }
        
        /**
         * @return the number of this branch
         */
        private int getNumber()
        {
            return number;
        }
        
        public void execute()
        {
            System.out.println("BEGINNING EXECUTE");
            if (currentIndex < branch.size())
            {
                Command currentCommand = getCommand(currentIndex);
                System.out.println("EXECUTING" + currentCommand);
                if (!hasStartedCommand)
                {
                    hasStartedCommand = true;
                    System.out.println("STARTING COMMAND");
                    Scheduler.getInstance().add(currentCommand);
                }
                if (currentCommand.isRunning())
                {
                    didCommandRun = true;
                    System.out.println("COMMAND IS RUNNING");
                }
                if (!currentCommand.isRunning() && didCommandRun)
                {
                    System.out.println("CANCELLING");
                    
                    incrementCurrentIndex();
                    didCommandRun = false;
                    hasStartedCommand = false;
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

        /**
         * Gets the required.
         * @return the required
         */
        public Subsystem getRequired()
        {
            return required;
        }

        /**
         * Sets required to a given value.
         * @param required the required to set
         *
         * @postcondition the required has been changed to required
         */
        public void setRequired(Subsystem required)
        {
            this.required = required;
        }
    
    }
}

//private ArrayList<ArrayList<Command>> branches;
//private ArrayList<Integer> currentCommandIndices;
//private ArrayList<Boolean> hasCommandStarted;
//private ArrayList<Boolean> wasCommandRunning;
//
//public BranchedCommandGroup()
//{
//    branches = new ArrayList<ArrayList<Command>>();
//    currentCommandIndices = new ArrayList<Integer>();
//    hasCommandStarted = new ArrayList<Boolean>();
//    wasCommandRunning = new ArrayList<Boolean>();
//    
//}
//public void addBranch(ArrayList<Command> branch)
//{
//    branches.add(branch);
//    currentCommandIndices.add(0);
//    hasCommandStarted.add(false);
//    wasCommandRunning.add(false);
//}
//
//public void execute()
//{
//    for (int i = 0; i < branches.size(); i++)
//    {
//        ArrayList<Command> currentBranch = branches.get(i);
//        double currentIndex = currentCommandIndices.get(i);
//        if (currentIndex != -1 && currentIndex < currentBranch.size())
//        {
//            Command currentCommand = currentBranch.get(currentCommandIndices.get(i));
//            if (!hasCommandStarted.get(i))
//            {
//                hasCommandStarted.set(i, true);
//                currentCommand.start();
//            }
//            if (currentCommand.isRunning())
//                wasCommandRunning.set(i, true);
//            if (!currentCommand.isRunning() && wasCommandRunning.get(i))
//            {
//                currentCommandIndices.set(i, currentCommandIndices.get(i)+1);
//                hasCommandStarted.set(i, false);
//                wasCommandRunning.set(i,  false);
//            }
//
//        }
//        else
//            currentCommandIndices.set(i, -1);
//    }
//}
//
//
///**
//* @return
//*/
//@Override
//public boolean isFinished()
//{
//    boolean isFinished = true;
//    for (int i : currentCommandIndices)
//    {
//        isFinished = isFinished && (i == -1);
//    }
//    return isFinished;
//}
