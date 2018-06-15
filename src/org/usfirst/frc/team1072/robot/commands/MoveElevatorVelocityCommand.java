package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

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
    public MoveElevatorVelocityCommand()
    {
        requires(Robot.el);
    }
    
    /**
     * Executes the command given a speed.
     * @param speed the speed by which the elevator will be moved
     */
    public void execute(double speed)
    {
        Robot.el.moveElevatorVelocity(speed);
    }
    
    /**
     * Determines whether the command has finished.
     * 
     * @return true if the command has finished; false otherwise
     */
    protected boolean isFinished()
    {
        // TODO Auto-generated method stub
        return true;
    }

}
