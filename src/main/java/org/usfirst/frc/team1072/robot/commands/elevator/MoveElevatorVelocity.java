package org.usfirst.frc.team1072.robot.commands.elevator;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;
import org.usfirst.frc.team1072.robot.RobotMap.DrivetrainConstants;
import org.usfirst.frc.team1072.robot.RobotMap.ElevatorConstants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents a command to continually move the elevator given a velocity.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class MoveElevatorVelocity extends Command
{
    /**
     * Creates a new MoveElevatorVelocity.
     */
    public MoveElevatorVelocity() { requires(Robot.el); }
    
    /**
     * Initializes the command, including necessary sensors.
     */
    public void initialize()
    {
        Robot.el.getBottomRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX, RobotMap.TIMEOUT);
    }
    /**
     * Executes the command using the speed from joystick input.
     */
    public void execute() 
    { 
        OI oi = OI.getInstance();
        if (Math.abs(oi.getDriverGamepad().getRightY()) > OI.BLACK_XBOX_ELEVATOR_DEADBAND)
        {
            boolean isDown = oi.getDriverGamepad().getRightY() < 0;
            boolean reverseBeyondLimit = Robot.el.getBottomRightTalon()
                    .getSelectedSensorPosition(DrivetrainConstants.POS_PID) <= ElevatorConstants.REVERSE_SOFT;
            if (isDown && !reverseBeyondLimit)
            {
                double speed = oi.getDriverGamepad().getRightY();
                if (Robot.el.getBottomRightTalon().getSelectedSensorPosition(ElevatorConstants.POS_PID) <= ElevatorConstants.SLOW_DOWN_POS)
                {
                    speed *= 0.01;
                }
                Robot.el.moveElevatorVelocity(speed);
            }
            else if (isDown && reverseBeyondLimit)
                Robot.el.moveElevatorVelocity(0);
            else
                Robot.el.moveElevatorVelocity(oi.getDriverGamepad().getRightY());
        }
        else
            Robot.el.moveElevatorVelocity(0);
    }
    
    /**
     * Determines whether the command has finished.
     * 
     * @return true if the command has finished; false otherwise
     */
    protected boolean isFinished() { return true; }
}
