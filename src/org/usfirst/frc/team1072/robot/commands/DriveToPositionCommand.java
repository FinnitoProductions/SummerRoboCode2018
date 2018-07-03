package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

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
    /**
     * Initializes the command, requiring the drive train.
     */
    public DriveToPositionCommand() { requires(Robot.dt); isJoystick = true; leftPosition = 0; rightPosition =0;}
    
    public DriveToPositionCommand(double currentLeftPosition, double currentRightPosition)
    {
        this.leftPosition = currentLeftPosition;
        this.rightPosition = currentRightPosition;
        isJoystick = false;
    }
    
    @Override
    public void initialize()
    {
        Robot.dt.selectProfileSlots(RobotMap.DT_POS_PID, RobotMap.PRIMARY_PID_INDEX);
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
            position = OI.getInstance().getGamepad().getLeftY() * RobotMap.TICKS_PER_REV * 3;
            Robot.dt.arcadeDrivePosition(position); 
        }
        else
            Robot.dt.arcadeDrivePosition(leftPosition, rightPosition);

        
        }
    
    /**
     * Determines whether the command has finished.
     * 
     * @return true if the command has finished; false otherwise
     */
    protected boolean isFinished() { return true; }
    
}
