package org.usfirst.frc.team1072.robot.commands.elevator;

import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Moves the elevator to a given position using PID.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class MoveElevatorPosition extends Command
{
    /**
     * The intended final position for the elevator.
     * 
     */
    private double position;
    
    /**
     * Creates a new MoveElevatorPosition.
     * @param position the position to move to
     */
    public MoveElevatorPosition(double position) 
    { 
        requires(Robot.el); 
    }
    
    /**
     * Initializes the command with necessary sensors.
     */
    public void initialize()
    {
        Robot.el.getBottomRightTalon().configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PRIMARY_PID_INDEX);
    }
    
    /**
     * Executes the command given the position to which the elevator should move. 
     * 
     */
    @Override
    public void execute() { Robot.el.moveElevatorPosition(position); }

    /**
     * Determines whether the command is complete.
     * 
     * @return true if the command is complete; false otherwise
     */
    protected boolean isFinished() { return true; }
}
