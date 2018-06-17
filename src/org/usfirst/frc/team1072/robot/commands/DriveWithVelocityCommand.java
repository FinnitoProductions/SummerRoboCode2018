package org.usfirst.frc.team1072.robot.commands;

import org.usfirst.frc.team1072.robot.OI;
import org.usfirst.frc.team1072.robot.Robot;
import org.usfirst.frc.team1072.robot.RobotMap;

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
    public DriveWithVelocityCommand() 
    { 
        requires(Robot.dt); 
    }
    
    /**
     * Executes the command to drive with a given velocity.
     * @param speed the speed at which the robot will drive
     * @param turn the amount by which the robot should turn
     */
    @Override
    public void execute() 
    { 
        OI oi = OI.getInstance();
        double driveSpeed = Robot.speedToEncoderUnits(oi.getGamepad().getLeftY() * RobotMap.MAX_DRIVE_SPEED); 
        double turnSpeed = Robot.speedToEncoderUnits(-1 * oi.getGamepad().getLeftX() * RobotMap.MAX_TURN_SPEED);
        Robot.dt.arcadeDriveVelocity(
                driveSpeed, 
                turnSpeed); 
    }
    
    /**
     * Determines whether the command has finished.
     */
    protected boolean isFinished() { return true; }

}
