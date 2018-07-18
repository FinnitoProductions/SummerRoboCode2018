package org.usfirst.frc.team1072.robot.commands.drivetrain;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.PigeonConstants;
import org.usfirst.frc.team1072.util.Conversions;
import org.usfirst.frc.team1072.util.Conversions.PositionUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives the robot to a given position using PID.
 * @author Finn Frankis
 * @version Jun 14, 2018
 */
public class DriveToPosition extends Command
{
    private double position;
    private int numExecutes;
    private int maxExecutes = 10;
    
    /**
     * Constructs a new DriveToPositionCommand.
     * @param position the final position for the robot in feet
     */
    public DriveToPosition (double position)
    {
        this.position = Conversions.convertPosition(PositionUnit.FEET, position, PositionUnit.ENCODER_UNITS);
    }
    
    /**
     * Initializes this command.
     */
    public void initialize()
    {
        initPosition();
        Robot.dt.setBothSensorPositions(0, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.setBoth(ControlMode.Position, position);
        Robot.dt.resetTalonCoefficients(RobotMap.PRIMARY_PID_INDEX);
        numExecutes = 0;
    }
    
    /**
     * Initializes the position part of this command (to be used if combined with a subsequent turn).
     */
    private void initPosition()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.setBothSensors(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.setTalonSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE, DrivetrainConstants.RIGHT_TALON_PHASE);
        
    }
    
    /**
     * Executes this command periodically.
     */
    public void execute()
    {
        if (numExecutes >= 0 && numExecutes < maxExecutes)
        {
            System.out.println("INCREMENTING");
            numExecutes++;
        }
        else
        {
            System.out.println("DONE");
            numExecutes = -1;
        }
        System.out.println("EXECUTING");
        Robot.dt.setBoth(ControlMode.Position, position);
    }
    /**
    * Determines whether this command has finished.
    * @return true if the error is within POS_ALLOWABLE_ERROR; false otherwise
    */
    @Override
    protected boolean isFinished()
    {
        if (numExecutes == -1)
        {
            return Robot.dt.isClosedLoopErrorWithin(RobotMap.PRIMARY_PID_INDEX, DrivetrainConstants.POS_ALLOWABLE_ERROR);
        }
        System.out.println("NOT FINISHED");
        return false;
    }
    
    public void end()
    {
        Robot.dt.getLeftTalon().set(ControlMode.PercentOutput, 0);
        Robot.dt.getRightTalon().set(ControlMode.PercentOutput, 0);
        System.out.println("FINISHING");
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

}
