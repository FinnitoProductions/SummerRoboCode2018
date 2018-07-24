package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;

import edu.wpi.first.wpilibj.command.Command;

/** 
 * Represents a wrapper class for a general type of position-based command.
 * @author Finn Frankis
 * @version Jul 21, 2018
 */
public abstract class PositionCommand extends Command
{
    private int numExecutes;
    private int maxExecutes;
    private double position;

    private double allowableError;
    
    /**
     * Constructs a new PositionCommand.
     * @param maxExecutes
     * @param desiredPos
     */
    public PositionCommand (int maxExecutes, double desiredPos)
    {
        numExecutes = 0;
        this.maxExecutes = maxExecutes;
        position = desiredPos;
        allowableError = DrivetrainConstants.POS_ALLOWABLE_ERROR;
    }
    
    /**
     * Increments the total number of executes.
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
    
    public void setNumExecutes(int numExecutes)
    {
        this.numExecutes = numExecutes;
    }
    
    /**
     * Gets the current position of the Talons.
     * @return
     */
    public double getCurrentPosition()
    {
        return (Robot.dt.getRightTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX) 
                + Robot.dt.getLeftTalon().getSelectedSensorPosition(RobotMap.PRIMARY_PID_INDEX)) / 2;
    }
    
    /**
     * Gets the position to where the robot is intended to travel.
     * @return the desired position
     */
    public double getDesiredPosition()
    {
        return position;
    }
    
    public boolean passedMaxExecutes()
    {
        return numExecutes > maxExecutes;
    }
    
    public boolean isWithinPercentOfDest(double percent)
    {
        return passedMaxExecutes() && 
        getCurrentPosition()/getDesiredPosition() >= percent;
    }
    
    public PositionCommand setAllowableError (double allowableError)
    {
        this.allowableError = allowableError;
        return this;
    }
    
    public double getAllowableError()
    {
        return allowableError;
    }
}
