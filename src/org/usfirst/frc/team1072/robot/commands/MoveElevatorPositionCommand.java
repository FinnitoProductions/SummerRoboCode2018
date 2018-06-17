package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Moves the elevator to a given position using PID.
 * @author Finn Frankis
 * @version 6/14/18
 */
public class MoveElevatorPositionCommand extends Command
{
    double position;
    /**
     * Creates a new command requiring the elevator subsystem.
     */
    public MoveElevatorPositionCommand(double position) 
    { 
        requires(Robot.el); 
    }

    /**
     * Executes the command given the position to which the elevator should move. 
     * @param position the position to move to
     */
    public void execute(double position) { Robot.el.moveElevatorPosition(position); }

    /**
     * Determines whether the command is complete.
     * 
     * @return true if the command is complete; false otherwise
     */
    protected boolean isFinished() { return true; }
}
