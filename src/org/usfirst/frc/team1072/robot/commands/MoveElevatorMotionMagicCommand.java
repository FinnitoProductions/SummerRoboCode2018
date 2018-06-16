package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Commands the elevator using the smooth, trapezoidal motion magic.
 * @author Finn Frankis
 * @version 6/15/18
 */
public class MoveElevatorMotionMagicCommand extends Command
{
    /**
     * Creates a new command requiring the elevator.
     */
    public MoveElevatorMotionMagicCommand() { requires(Robot.el); }
    
    /**
     * Executes the command, moving the robot to a given position using motion magic.
     * @param targetPos the position to which the robot should be moved
     */
    public void execute(double targetPos)
    {
        Robot.el.moveElevatorMotionMagic(targetPos);
    }

   /**
    * Determines whether the command is complete.
    * 
    * @return true if the command has finished; false otherwise
    */
    protected boolean isFinished() {
        return true;
    }
    
}
