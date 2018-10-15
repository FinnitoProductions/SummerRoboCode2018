package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Finn Frankis
 * @version Jul 21, 2018
 */
public abstract class PositionCommand extends Command
{
    /**
     * The number of times which this command has currently been executed.
     */
    private int numExecutes;
    
    /**
     * The minimum number of times this command must execute to be marked as completed. This prevents
     * isFinished() from returning true immediately due to a delay in error increase.
     */
    private int maxExecutes;
    
    /**
     * The desired position for the robot.
     */
    private double position;
    
    /**
     * The allowable error for this command (the range for which it can be considered completed)
     */
    private double allowableError;
    
    /**
     * Constructs a new PositionCommand.java
     * @param maxExecutes the minimum number of times this command must execute to be marked as completed
     * @param desiredPos the desired position for this command
     */
    public PositionCommand (int maxExecutes, double desiredPos)
    {
        numExecutes = 0;
        this.maxExecutes = maxExecutes;
        position = desiredPos;
        allowableError = Drivetrain.POS_ALLOWABLE_ERROR;
    }
    
    /**
     * Increments the total number of executes, increasing it by 1.
     */
    public void incrementNumExecutes()
    {
        numExecutes++;
    }
    /**
     * Gets the total number of executes.
     * @return the total number of times the command has executed
     */
    public int getNumExecutes()
    {
        return numExecutes;
    }
    
    /**
     * Gets the maximum number of executes required to end.
     * @return the maximum number of executes required
     */
    public int getMaxExecutes()
    {
        return maxExecutes;
    }
    
    /**
     * Sets the total number of executes.
     * @param numExecutes the total number of executes
     */
    public void setNumExecutes(int numExecutes)
    {
        this.numExecutes = numExecutes;
    }
    
    /**
     * Gets the current position of the Talons.
     * @return the current Talon position
     */
    public double getCurrentPosition()
    {
        return (Robot.dt.getRightMaster().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX) 
                + Robot.dt.getLeftMaster().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX)) / 2;
    }
    
    /**
     * Gets the position to where the robot is intended to travel.
     * @return the desired position
     */
    public double getDesiredPosition()
    {
        return position;
    }
    
    /**
     * Determines whether this command has executed beyond maxExecutes.
     * @return true if it has executed beyond the minimum required amount; false otherwise
     */
    public boolean passedMaxExecutes()
    {
        return numExecutes > maxExecutes;
    }
    
    /**
     * Determines whether the current position is within a given percentage of the destination.
     * @param percent the percentage to be checked [0, 1]
     * @return true if the current position is within the percentage; false otherwise
     */
    public boolean isWithinPercentOfDest(double percent)
    {
        return passedMaxExecutes() && 
        getCurrentPosition()/getDesiredPosition() >= percent;
    }
    
    /**
     * Sets the allowable error of this command.
     * @param allowableError the allowable error to be set to
     * @return this command
     */
    public PositionCommand setAllowableError (double allowableError)
    {
        this.allowableError = allowableError;
        return this;
    }
    
    /**
     * Gets the allowable error of this command.
     * @return the allowable error
     */
    public double getAllowableError()
    {
        return allowableError;
    }
}