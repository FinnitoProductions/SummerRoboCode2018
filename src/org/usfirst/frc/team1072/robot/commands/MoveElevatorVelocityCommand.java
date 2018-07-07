package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a command to continually move the elevator given a velocity.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class MoveElevatorVelocityCommand extends Command
{
    /**
     * Creates a new command requiring the elevator.
     */
    public MoveElevatorVelocityCommand() { requires(Robot.el); }
    
    /**
     * Initializes the command, including necessary sensors.
     */
    public void initialize()
    {
        Robot.el.getBottomRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
    }
    /**
     * Executes the command given a speed.
     * @param speed the speed by which the elevator will be moved
     */
    public void execute() 
    { 
        OI oi = OI.getInstance();
        if (Math.abs(oi.getGamepad().getRightY()) > OI.BLACK_XBOX_DEADBAND)
        {
            boolean isDown = oi.getGamepad().getRightY() < 0;
            boolean reverseBeyondLimit = Robot.el.getBottomRightTalon()
                    .getSelectedSensorPosition(DrivetrainConstants.POS_PID) <= ElevatorConstants.REVERSE_SOFT;
            if (isDown && !reverseBeyondLimit)
            {
                double speed = oi.getGamepad().getRightY();
                if (Robot.el.getBottomRightTalon().getSelectedSensorPosition(ElevatorConstants.POS_PID) <= ElevatorConstants.SLOW_DOWN_POS)
                {
                    speed *= 0.01;
                }
                Robot.el.moveElevatorVelocity(speed);
            }
            else if (isDown && reverseBeyondLimit)
                Robot.el.moveElevatorVelocity(0);
            else
                Robot.el.moveElevatorVelocity(oi.getGamepad().getRightY());
        }
        else
            Robot.el.moveElevatorVelocity(0);
        
        SmartDashboard.putNumber("Joystick Value", oi.getGamepad().getRightY());
    }
    
    /**
     * Determines whether the command has finished.
     * 
     * @return true if the command has finished; false otherwise
     */
    protected boolean isFinished() { return true; }
}
