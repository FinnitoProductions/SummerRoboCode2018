package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.util.Position;
import org.usfirst.frc.team1072.util.Position.PositionUnit;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Commands the robot to move to a given position using PID.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class DriveToPositionCommand extends Command
{
    double leftPosition;
    double rightPosition;
    boolean isJoystick;
    private boolean checkIsFinished;
    private double numExecutes;
    /**
     * Initializes the command, requiring the drive train.
     */
    public DriveToPositionCommand() { requires(Robot.dt); isJoystick = true; leftPosition = 0; rightPosition =0;}
    
    public DriveToPositionCommand (double destPosition)
    {
        requires(Robot.dt);
        isJoystick = false;
        leftPosition = destPosition;
        rightPosition = destPosition;
    }
    public DriveToPositionCommand(double currentLeftPosition, double currentRightPosition)
    {
        requires(Robot.dt);
        this.leftPosition = currentLeftPosition;
        this.rightPosition = currentRightPosition;
        isJoystick = false;
    }
    
    @Override
    /**
     * Initializes the command.
     */
    public void initialize()
    {
        Robot.dt.selectProfileSlots(DrivetrainConstants.POS_PID, RobotMap.PRIMARY_PID_INDEX);
        
        Robot.dt.getLeftTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        
        Robot.dt.getLeftTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        Robot.dt.getRightTalon().setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
        
        Robot.dt.resetTalonCoefficients(DrivetrainConstants.POS_PID);
        
        System.out.println("INITIALIZING");
        numExecutes = 0;
        checkIsFinished = false;
    }
    /**
     * Executes the command, moving the robot to a given position.
     * @param position the position to which the robot should move
     */
    
    public void execute() 
    { 
        double position;
        if (isJoystick)
        {
            position = new Position(PositionUnit.FEET, OI.getInstance().getGamepad().getLeftY() * 5, DrivetrainConstants.WHEELDIAMETER).getEncoderUnits();
            Robot.dt.arcadeDrivePosition(position); 
        }
        else
        {
            if (numExecutes >= 0 && numExecutes < 20)
                numExecutes++;
            if (numExecutes >= 20)
            {
                checkIsFinished = true;
                numExecutes = -1;
            }
            Robot.dt.arcadeDrivePosition(leftPosition, rightPosition);
        }
    }
    /**
     * Determines whether the command has finished.
     * 
     * @return true if the command has finished; false otherwise
     */
    protected boolean isFinished() 
    {
        if (checkIsFinished)
        {
            return Robot.dt.getLeftTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX) < RobotMap.MOTION_PROFILE_END_ERROR
                    && Robot.dt.getRightTalon().getClosedLoopError(RobotMap.PRIMARY_PID_INDEX) < RobotMap.MOTION_PROFILE_END_ERROR;
        }
            
        return false;

    }
    
}
