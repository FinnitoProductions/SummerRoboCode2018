package org.usfirst.frc.team1072.robot.commands.intake;

import org.usfirst.frc.team1072.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Represents the command to control the process of intaking and outtaking cubes.
 * 
 * @author Finn Frankis
 * @version 6/14/18
 */
public class IntakeOuttakeIndefinite extends Command
{
    private double speed;
    /**
     * Constructs a new command requiring the intake.
     */
    public IntakeOuttakeIndefinite(double speed) 
    { 
        requires(Robot.intake); 
        this.speed = speed;
    }

    /**
     * Executes the intake/outtake command given a left and right speed, using the trigger.
     */
    public void execute() 
    {
        Robot.intake.intakeOuttakeCube(-speed);
    }
    
    /**
     * Calls when the command is interrupted or cancelled.
     */
    @Override
    public void end()
    {
        Robot.intake.intakeOuttakeCube(0);
    }
    
    /**
     * Determines whether this command has finished.
     */
    protected boolean isFinished() { return false; }

}
