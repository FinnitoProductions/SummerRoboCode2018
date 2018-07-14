package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Commands the robot to move to a given position using PID.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class DriveToPositionCommand extends Command
{
    private double leftPosition;
    private double rightPosition;
    private boolean isJoystick;
    private boolean hasInitialized;
    private double numExecutes;
    
    /**
     * Initializes the command, requiring the drive train.
     */
    public DriveToPositionCommand() { requires(Robot.dt); isJoystick = true; leftPosition = 0; rightPosition =0;}
    
    /**
     * Constructs a new DriveToPositionCommand given an encoder position.
     * @param currentLeftPosition the intended left position 
     * @param currentRightPosition the intended right position 
     */
    public DriveToPositionCommand (double destPosition)
    {
        requires(Robot.dt);
        isJoystick = false;
        leftPosition = destPosition;
        rightPosition = destPosition;
        hasInitialized = false;
    }
    
    /**
     * Constructs a new DriveToPositionCommand given left and right encoder positions.
     * @param currentLeftPosition the intended left position 
     * @param currentRightPosition the intended right position 
     */
    public DriveToPositionCommand(double currentLeftPosition, double currentRightPosition)
    {
        requires(Robot.dt);
        this.leftPosition = currentLeftPosition;
        this.rightPosition = currentRightPosition;
        isJoystick = false;
        hasInitialized = false;
    }
    
    @Override
    /**
     * Initializes the command.
     */
    public void initialize()
    {
        Robot.dt.getLeftTalon().selectProfileSlot(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        Robot.dt.getRightTalon().selectProfileSlot(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);

        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSelectedFeedbackSensor(
                FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        
        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(
                FeedbackDevice.None, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSelectedFeedbackSensor(
                FeedbackDevice.None, RobotMap.AUXILIARY_PID_INDEX, RobotMap.TIMEOUT);
        
        Robot.dt.setTalonSensorPhase(DrivetrainConstants.LEFT_TALON_PHASE, 
                DrivetrainConstants.RIGHT_TALON_PHASE);
        Robot.dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        
        Robot.dt.resetTalonCoefficients(DrivetrainConstants.POS_PID);
        Robot.dt.arcadeDrivePosition(leftPosition, rightPosition);
        hasInitialized = false;
        System.out.println("INITIALIZING POSITION COMMAND");

    }
    /**
     * Executes the command, moving the robot to a given position.
     * @param position the position to which the robot should move
     */
    
    public void execute() 
    { 
        if (numExecutes >= 0 && numExecutes < 3)
            numExecutes++;
        else
        {
            hasInitialized = true;
            numExecutes = -1;
        }
        Robot.dt.arcadeDrivePosition(leftPosition, rightPosition);
    }
    
    /**
     * Determines whether the command has finished.
     * 
     * @return true if the command has finished; false otherwise
     */
    protected boolean isFinished() 
    {
        /*if (hasInitialized)
            return Robot.dt.getLeftTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX) < RobotMap.MOTION_PROFILE_END_ERROR
                    && Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX) < RobotMap.MOTION_PROFILE_END_ERROR;*/
        return false;
    }
    
}
