package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Commands the drivetrain to drive with a given velocity. Vel PID
 * 
 * @author Finn Frankis
 * @version 6/11/18
 */
public class DriveWithVelocityCommand extends Command
{

    /**
     * Creates a new DriveWithVelocityCommand object requiring the Drivetrain.
     */
    public DriveWithVelocityCommand() { requires(Robot.dt); }
    
    /**
     * Executes the command to drive with a given velocity.
     * @param speed the speed at which the robot will drive
     * @param turn the amount by which the robot should turn
     */
    @Override
    public void execute(double speed, double turn) { Robot.dt.arcadeDriveVelocity(speed, turn); }
    
    /**
     * Determines whether the command has finished.
     */
    protected boolean isFinished() { return true; }

}
